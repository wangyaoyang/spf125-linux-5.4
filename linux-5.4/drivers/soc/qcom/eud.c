// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2020 The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#ifdef CONFIG_EUD_EXTCON_SUPPORT
#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#endif
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#ifdef CONFIG_EUD_EXTCON_SUPPORT
#include <linux/power_supply.h>
#endif
#include <linux/qcom_scm.h>

#define EUD_ENABLE_CMD 1
#define EUD_DISABLE_CMD 0

#define EUD_REG_COM_TX_ID	0x0000
#define EUD_REG_COM_TX_LEN	0x0004
#define EUD_REG_COM_TX_DAT	0x0008
#define EUD_REG_COM_RX_ID	0x000C
#define EUD_REG_COM_RX_LEN	0x0010
#define EUD_REG_COM_RX_DAT	0x0014
#define EUD_REG_EUD_EN2		0x0000
#define EUD_REG_INT1_EN_MASK	0x0024
#define EUD_REG_INT_STATUS_1	0x0044
#define EUD_REG_CTL_OUT_1	0x0074
#define EUD_REG_VBUS_INT_CLR	0x0080
#define EUD_REG_CHGR_INT_CLR	0x0084

#define EUD_DEV_ID_1		0x0004
#define EUD_DEV_ID_2		0x0008
#define EUD_DEV_ID_3		0x000c
#define EUD_REG_CSR_EUD_EN	0x0014
#define EUD_REG_SW_ATTACH_DET	0x0018

#define EUD_EUD_EN2		0x1000

#define EUD_INT_RX		BIT(0)
#define EUD_INT_TX		BIT(1)
#define EUD_INT_VBUS		BIT(2)
#define EUD_INT_CHGR		BIT(3)
#define EUD_INT_SAFE_MODE	BIT(4)
#define EUD_INT_ALL		(EUD_INT_RX | EUD_INT_TX | \
				EUD_INT_VBUS | EUD_INT_CHGR | \
				EUD_INT_SAFE_MODE)

#define EUD_NR			1
#define EUD_CONSOLE		NULL
#define UART_ID			0x90
#define MAX_FIFO_SIZE		14

#ifdef CONFIG_EUD_EXTCON_SUPPORT
struct eud_chip {
	struct device			*dev;
	int				eud_irq;
	unsigned int			extcon_id;
	unsigned int			int_status;
	bool				usb_attach;
	bool				chgr_enable;
	void __iomem			*eud_reg_base;
	struct extcon_dev		*extcon;
	struct uart_port		port;
	struct work_struct		eud_work;
	struct power_supply		*batt_psy;
	bool				secure_eud_en;
	bool				need_phy_clk_vote;
	struct clk			*eud_ahb2phy_clk;
};
#else
struct eud_chip {
	struct device			*dev;
	int				eud_irq;
	void __iomem			*eud_reg_base;
	void __iomem			*eud_mode_mgr;
	struct uart_port		port;
	bool				secure_eud_en;
	bool				need_phy_clk_vote;
	phys_addr_t			eud_mode_mgr2_phys_base;
	struct clk			*eud_ahb2phy_clk;
};
#endif

#ifdef CONFIG_EUD_EXTCON_SUPPORT
static const unsigned int eud_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_CHG_USB_SDP,
	EXTCON_NONE,
};
#endif

/*
 * On the kernel command line specify eud.enable=1 to enable EUD.
 * EUD is disabled by default.
 */
static int enable;
static bool eud_ready;
static struct platform_device *eud_private;

static void enable_eud(struct platform_device *pdev)
{
	struct eud_chip *priv = platform_get_drvdata(pdev);
#ifdef CONFIG_EUD_EXTCON_SUPPORT
	int ret;
#endif

	/* set EUD_EN bit */
	writel_relaxed(BIT(0), priv->eud_mode_mgr + EUD_EUD_EN2);
	/* write into CSR to enable EUD */
	writel_relaxed(BIT(0), priv->eud_mode_mgr + EUD_REG_CSR_EUD_EN);

	/* Enable vbus, chgr & safe mode warning interrupts */
	writel_relaxed(EUD_INT_VBUS | EUD_INT_CHGR | EUD_INT_SAFE_MODE,
			priv->eud_reg_base + EUD_REG_INT1_EN_MASK);

	/* write EUD DEV_ID */
	writel_relaxed(0x80, priv->eud_mode_mgr + EUD_DEV_ID_1);
	writel_relaxed(0x30, priv->eud_mode_mgr + EUD_DEV_ID_2);
	writel_relaxed(0x79, priv->eud_mode_mgr + EUD_DEV_ID_3);

#ifdef CONFIG_EUD_EXTCON_SUPPORT
	/* Enable secure eud if supported */
	if (priv->secure_eud_en) {
		ret = qcom_scm_io_writel(priv->eud_mode_mgr2_phys_base +
				   EUD_REG_EUD_EN2, EUD_ENABLE_CMD);
		if (ret)
			dev_err(&pdev->dev,
			"qcom_scm_io_writel failed with rc:%d\n", ret);
	}
#endif
	/* Ensure Register Writes Complete */
	wmb();

	/*
	 * Set the default cable state to usb connect and charger
	 * enable
	 */
#ifdef CONFIG_EUD_EXTCON_SUPPORT
	extcon_set_state_sync(priv->extcon, EXTCON_USB, true);
	extcon_set_state_sync(priv->extcon, EXTCON_CHG_USB_SDP, true);
#endif

	dev_dbg(&pdev->dev, "%s: EUD is Enabled\n", __func__);
}

static void disable_eud(struct platform_device *pdev)
{
	struct eud_chip *priv = platform_get_drvdata(pdev);
#ifdef CONFIG_EUD_EXTCON_SUPPORT
	int ret;
#endif

	/* Unset EUD_EN bit */
	writel_relaxed(0, priv->eud_mode_mgr + EUD_EUD_EN2);
	/* write into CSR to disable EUD */
	writel_relaxed(0, priv->eud_mode_mgr + EUD_REG_CSR_EUD_EN);

#ifdef CONFIG_EUD_EXTCON_SUPPORT
	/* Disable secure eud if supported */
	if (priv->secure_eud_en) {
		ret = qcom_scm_io_writel(priv->eud_mode_mgr2_phys_base +
				   EUD_REG_EUD_EN2, EUD_DISABLE_CMD);
		if (ret)
			dev_err(&pdev->dev,
			"qcom_scm_io_write failed with rc:%d\n", ret);
	}
#endif

	dev_dbg(&pdev->dev, "%s: EUD Disabled!\n", __func__);
}

static int param_eud_set(const char *val, const struct kernel_param *kp)
{
	int enable = 0;

	if (sscanf(val, "%du", &enable) != 1)
		return -EINVAL;

	if (enable != EUD_ENABLE_CMD && enable != EUD_DISABLE_CMD)
		return -EINVAL;

	*((uint *)kp->arg) = enable;
	if (!eud_ready)
		return 0;

	if (enable == EUD_ENABLE_CMD) {
		pr_debug("%s: Enbling EUD\n", __func__);
		enable_eud(eud_private);
	} else if (enable == EUD_DISABLE_CMD) {
		pr_debug("%s: Disabling EUD\n", __func__);
		disable_eud(eud_private);
	}

	return 0;
}

static const struct kernel_param_ops eud_param_ops = {
	.set = param_eud_set,
	.get = param_get_int,
};

module_param_cb(enable, &eud_param_ops, &enable, 0644);

#ifdef CONFIG_EUD_EXTCON_SUPPORT
static bool is_batt_available(struct eud_chip *chip)
{
	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (!chip->batt_psy)
		return false;

	return true;
}

static void eud_event_notifier(struct work_struct *eud_work)
{
	struct eud_chip *chip = container_of(eud_work, struct eud_chip,
					eud_work);
	union power_supply_propval pval;

	if (chip->int_status == EUD_INT_VBUS)
		extcon_set_state_sync(chip->extcon, chip->extcon_id,
					chip->usb_attach);
	else if (chip->int_status == EUD_INT_CHGR) {
		if (is_batt_available(chip)) {
			pval.intval = chip->chgr_enable ? -EINVAL :
				chip->chgr_enable;
			power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
		}
	}
}

static void usb_attach_detach(struct eud_chip *chip)
{
	u32 reg;

	chip->extcon_id = EXTCON_USB;
	/* read ctl_out_1[4] to find USB attach or detach event */
	reg = readl_relaxed(chip->eud_reg_base + EUD_REG_CTL_OUT_1);
	if (reg & BIT(4))
		chip->usb_attach = true;
	else
		chip->usb_attach = false;

	schedule_work(&chip->eud_work);

	/* set and clear vbus_int_clr[0] to clear interrupt */
	writel_relaxed(BIT(0), chip->eud_reg_base + EUD_REG_VBUS_INT_CLR);
	/* Ensure Register Writes Complete */
	wmb();
	writel_relaxed(0, chip->eud_reg_base + EUD_REG_VBUS_INT_CLR);
}

static void chgr_enable_disable(struct eud_chip *chip)
{
	u32 reg;

	chip->extcon_id = EXTCON_CHG_USB_SDP;
	/* read ctl_out_1[6] to find charger enable or disable event */
	reg = readl_relaxed(chip->eud_reg_base + EUD_REG_CTL_OUT_1);
	if (reg & BIT(6))
		chip->chgr_enable = true;
	else
		chip->chgr_enable = false;

	schedule_work(&chip->eud_work);

	/* set and clear chgr_int_clr[0] to clear interrupt */
	writel_relaxed(BIT(0), chip->eud_reg_base + EUD_REG_CHGR_INT_CLR);
	/* Ensure Register Writes Complete */
	wmb();
	writel_relaxed(0, chip->eud_reg_base + EUD_REG_CHGR_INT_CLR);
}
#endif

static void pet_eud(struct eud_chip *chip)
{
	u32 reg;

	/* read sw_attach_det[0] to find attach/detach event */
	reg = readl_relaxed(chip->eud_mode_mgr + EUD_REG_SW_ATTACH_DET);
	if (reg & BIT(0)) {
		/* Detach & Attach pet for EUD */
		writel_relaxed(0, chip->eud_mode_mgr + EUD_REG_SW_ATTACH_DET);
		/* Ensure Register Writes Complete */
		wmb();
		/* Delay to make sure detach pet is done before attach pet */
		udelay(100);
		writel_relaxed(BIT(0), chip->eud_mode_mgr +
					EUD_REG_SW_ATTACH_DET);
		/* Ensure Register Writes Complete */
		wmb();
	} else {
		/* Attach pet for EUD */
		writel_relaxed(BIT(0), chip->eud_mode_mgr +
					EUD_REG_SW_ATTACH_DET);
		/* Ensure Register Writes Complete */
		wmb();
	}
}

static unsigned int eud_tx_empty(struct uart_port *port)
{
	u32 reg;

	/* read status register and cross check for Tx interrupt */
	reg = readl_relaxed(port->membase + EUD_REG_INT_STATUS_1);
	if (reg & EUD_INT_TX)
		return TIOCSER_TEMT;
	else
		return 0;
}

static void eud_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* Nothing to set */
}

static unsigned int eud_get_mctrl(struct uart_port *port)
{
	/* Nothing to get */
	return 0;
}

static void eud_stop_tx(struct uart_port *port)
{
	/* Disable Tx interrupt */
	writel_relaxed(~EUD_INT_TX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static void eud_start_tx(struct uart_port *port)
{
	/* Enable Tx interrupt */
	writel_relaxed(EUD_INT_TX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static void eud_stop_rx(struct uart_port *port)
{
	/* Disable Rx interrupt */
	writel_relaxed(~EUD_INT_RX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static int eud_startup(struct uart_port *port)
{
	/* Enable Rx interrupt */
	writel_relaxed(EUD_INT_RX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
	return 0;
}

static void eud_shutdown(struct uart_port *port)
{
	/* Disable both Tx & Rx interrupts */
	writel_relaxed(~EUD_INT_TX | ~EUD_INT_RX,
			port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static const char *eud_type(struct uart_port *port)
{
	return (port->type == PORT_EUD_UART) ? "EUD UART" : NULL;
}

static int eud_request_port(struct uart_port *port)
{
	/* Nothing to request */
	return 0;
}

static void eud_release_port(struct uart_port *port)
{
	/* Nothing to release */
}

static void eud_config_port(struct uart_port *port, int flags)
{
	/* set port type, clear Tx and Rx interrupts */
	port->type = PORT_EUD_UART;
	writel_relaxed(~EUD_INT_TX | ~EUD_INT_RX,
			port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static int eud_verify_port(struct uart_port *port,
				       struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_EUD_UART)
		return -EINVAL;
	return 0;
}

/* serial functions supported */
static const struct uart_ops eud_uart_ops = {
	.tx_empty	= eud_tx_empty,
	.set_mctrl	= eud_set_mctrl,
	.get_mctrl	= eud_get_mctrl,
	.stop_tx	= eud_stop_tx,
	.start_tx	= eud_start_tx,
	.stop_rx	= eud_stop_rx,
	.startup	= eud_startup,
	.shutdown	= eud_shutdown,
	.type		= eud_type,
	.release_port	= eud_release_port,
	.request_port	= eud_request_port,
	.config_port	= eud_config_port,
	.verify_port	= eud_verify_port,
};

static struct uart_driver eud_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "msm-eud",
	.dev_name	= "ttyEUD",
	.nr		= EUD_NR,
	.cons		= EUD_CONSOLE,
};

static void eud_uart_rx(struct eud_chip *chip)
{
	struct uart_port *port = &chip->port;
	u32 reg;
	unsigned int len;
	unsigned char ch, flag;
	int i;

	reg = readl_relaxed(chip->eud_reg_base + EUD_REG_COM_RX_ID);
	if (reg != UART_ID) {
		dev_dbg(chip->dev, "Rx interrupt isn't for us\n");
		return;
	}
	/* Read Rx Len & Data registers */
	spin_lock(&port->lock);
	len = readl_relaxed(chip->eud_reg_base + EUD_REG_COM_RX_LEN);
	for (i = 0; i < len; i++) {
		ch = readl_relaxed(chip->eud_reg_base + EUD_REG_COM_RX_DAT);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (uart_handle_sysrq_char(port, ch))
			continue;
		uart_insert_char(port, 0, 0, ch, flag);
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
}

static void eud_uart_tx(struct eud_chip *chip)
{
	struct uart_port *port = &chip->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int len;
	u32 reg;

	writel_relaxed(UART_ID, chip->eud_reg_base + EUD_REG_COM_TX_ID);
	reg = readl_relaxed(chip->eud_reg_base + EUD_REG_COM_TX_ID);
	if (reg != UART_ID) {
		dev_dbg(chip->dev, "Tx interrupt isn't for us\n");
		return;
	}
	/* Write to Tx Len & Data registers */
	spin_lock(&port->lock);
	len = uart_circ_chars_pending(xmit);
	if (len > 0) {
		if (len > port->fifosize)
			len = port->fifosize;
		while (len--) {
			writel_relaxed(xmit->buf[xmit->tail],
			       port->membase + EUD_REG_COM_TX_DAT);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
		}
	}
	spin_unlock(&port->lock);
}

static irqreturn_t handle_eud_irq(int irq, void *data)
{
	struct eud_chip *chip = data;
	u32 reg;
	u32 int_mask_en1 = readl_relaxed(chip->eud_reg_base +
					EUD_REG_INT1_EN_MASK);

	/* read status register and find out which interrupt triggered */
	reg = readl_relaxed(chip->eud_reg_base + EUD_REG_INT_STATUS_1);
	switch (reg & EUD_INT_ALL) {
	case EUD_INT_RX:
		dev_dbg(chip->dev, "EUD RX Interrupt is received\n");
		eud_uart_rx(chip);
		break;
	case EUD_INT_TX:
		if (EUD_INT_TX & int_mask_en1) {
			dev_dbg(chip->dev, "EUD TX Interrupt is received\n");
			eud_uart_tx(chip);
		}
		break;
#ifdef CONFIG_EUD_EXTCON_SUPPORT
	case EUD_INT_VBUS:
		dev_dbg(chip->dev, "EUD VBUS Interrupt is received\n");
		chip->int_status = EUD_INT_VBUS;
		usb_attach_detach(chip);
		break;
	case EUD_INT_CHGR:
		dev_dbg(chip->dev, "EUD CHGR Interrupt is received\n");
		chip->int_status = EUD_INT_CHGR;
		chgr_enable_disable(chip);
		break;
#endif
	case EUD_INT_SAFE_MODE:
		dev_dbg(chip->dev, "EUD SAFE MODE Interrupt is received\n");
		pet_eud(chip);
		break;
	default:
		dev_dbg(chip->dev, "Unknown EUD Interrupt is received\n");
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int msm_eud_suspend(struct device *dev)
{
	struct eud_chip *chip = dev_get_drvdata(dev);

	if (chip->need_phy_clk_vote && chip->eud_ahb2phy_clk)
		clk_disable_unprepare(chip->eud_ahb2phy_clk);

	return 0;
}

static int msm_eud_resume(struct device *dev)
{
	struct eud_chip *chip = dev_get_drvdata(dev);
	int ret = 0;

	if (chip->need_phy_clk_vote && chip->eud_ahb2phy_clk) {
		ret = clk_prepare_enable(chip->eud_ahb2phy_clk);
		if (ret)
			dev_err(chip->dev, "%s failed to vote ahb2phy clk %d\n",
					__func__, ret);
	}

	return ret;
}
#endif

static int msm_eud_probe(struct platform_device *pdev)
{
	struct eud_chip *chip;
	struct uart_port *port;
	struct resource *res;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	platform_set_drvdata(pdev, chip);

	chip->dev = &pdev->dev;

#ifdef CONFIG_EUD_EXTCON_SUPPORT
	chip->extcon = devm_extcon_dev_allocate(&pdev->dev, eud_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		dev_err(chip->dev, "%s: failed to allocate extcon device\n",
					__func__);
		return PTR_ERR(chip->extcon);
	}

	ret = devm_extcon_dev_register(&pdev->dev, chip->extcon);
	if (ret) {
		dev_err(chip->dev, "%s: failed to register extcon device\n",
					__func__);
		return ret;
	}
#endif

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "eud_base");
	if (!res) {
		dev_err(chip->dev, "%s: failed to get resource eud_base\n",
					__func__);
		return -ENOMEM;
	}

	chip->eud_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chip->eud_reg_base))
		return PTR_ERR(chip->eud_reg_base);

	/* EUD_MODE_MANAGER for DEV_ID*/
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "eud_mode_mgr");
	if (!res) {
		dev_err(chip->dev, "%s: failed to get resource eud_mode_mgr\n",
					__func__);
		return -ENOMEM;
	}

	chip->eud_mode_mgr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chip->eud_mode_mgr))
		return PTR_ERR(chip->eud_mode_mgr);

#ifdef CONFIG_EUD_EXTCON_SUPPORT
	/* If secure EUD is supported, use EUD_MODE_MANAGER2 for EUD_EN
	 * EUD_MODE_MANAGER size to be reduced to 0x1000 in dtsi
	 */
	chip->secure_eud_en = of_property_read_bool(pdev->dev.of_node,
			      "qcom,secure-eud-en");
	if (chip->secure_eud_en) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "eud_mode_mgr2");
		if (!res) {
			dev_err(chip->dev,
			"%s: failed to get resource eud_mode_mgr2\n",
			__func__);
			return -ENOMEM;
		}

		chip->eud_mode_mgr2_phys_base = res->start;
	}
#endif

	chip->need_phy_clk_vote = of_property_read_bool(pdev->dev.of_node,
			      "qcom,eud-clock-vote-req");
	if (chip->need_phy_clk_vote) {
		chip->eud_ahb2phy_clk = devm_clk_get(&pdev->dev,
						     "eud_ahb2phy_clk");
		if (IS_ERR(chip->eud_ahb2phy_clk)) {
			ret = PTR_ERR(chip->eud_ahb2phy_clk);
			return ret;
		}

		ret = clk_prepare_enable(chip->eud_ahb2phy_clk);
		if (ret)
			return ret;
	}

	chip->eud_irq = platform_get_irq_byname(pdev, "eud_irq");
	ret = devm_request_irq(&pdev->dev, chip->eud_irq, handle_eud_irq,
				IRQF_TRIGGER_HIGH, "eud_irq", chip);
	if (ret) {
		dev_err(chip->dev, "request failed for eud irq\n");
		goto error;
	}

	device_init_wakeup(&pdev->dev, true);
	enable_irq_wake(chip->eud_irq);

#ifdef CONFIG_EUD_EXTCON_SUPPORT
	INIT_WORK(&chip->eud_work, eud_event_notifier);
#endif

	pdev->id = 0;
	port = &chip->port;
	port->line = pdev->id;
	port->type = PORT_EUD_UART;
	port->dev = chip->dev;
	port->fifosize = MAX_FIFO_SIZE;
	port->iotype = SERIAL_IO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->membase = chip->eud_reg_base;
	port->irq = chip->eud_irq;
	port->ops = &eud_uart_ops;

	ret = uart_add_one_port(&eud_uart_driver, port);
	if (ret) {
		dev_err(chip->dev, "failed to add uart port!\n");
		goto error;
	}

	eud_private = pdev;
	eud_ready = true;

	/* Enable EUD */
	if (enable)
		enable_eud(pdev);

	return 0;

error:
	if (chip->need_phy_clk_vote && chip->eud_ahb2phy_clk)
		clk_disable_unprepare(chip->eud_ahb2phy_clk);

	return ret;
}

static int msm_eud_remove(struct platform_device *pdev)
{
	struct eud_chip *chip = platform_get_drvdata(pdev);
	struct uart_port *port = &chip->port;

	uart_remove_one_port(&eud_uart_driver, port);
	device_init_wakeup(chip->dev, false);
	if (chip->need_phy_clk_vote)
		clk_disable_unprepare(chip->eud_ahb2phy_clk);

	return 0;
}

static const struct of_device_id msm_eud_dt_match[] = {
	{.compatible = "qcom,msm-eud"},
	{},
};
MODULE_DEVICE_TABLE(of, msm_eud_dt_match);

static const struct dev_pm_ops msm_eud_dev_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(msm_eud_suspend, msm_eud_resume)
};

static struct platform_driver msm_eud_driver = {
	.probe		= msm_eud_probe,
	.remove		= msm_eud_remove,
	.driver		= {
		.name		= "msm-eud",
		.pm = &msm_eud_dev_pm_ops,
		.of_match_table = msm_eud_dt_match,
	},
};

static int __init msm_eud_init(void)
{
	int ret;

	ret = uart_register_driver(&eud_uart_driver);
	if (ret) {
		pr_err("%s: Failed to register EUD UART driver\n",
			__func__);
		return ret;
	}

	ret = platform_driver_register(&msm_eud_driver);
	if (ret) {
		pr_err("%s: Failed to register EUD driver\n",
			__func__);
		uart_unregister_driver(&eud_uart_driver);
		return ret;
	}

	return 0;
}
module_init(msm_eud_init);

static void __exit msm_eud_exit(void)
{
	platform_driver_unregister(&msm_eud_driver);
	uart_unregister_driver(&eud_uart_driver);
}
module_exit(msm_eud_exit);

MODULE_DESCRIPTION("QTI EUD driver");
MODULE_LICENSE("GPL v2");
