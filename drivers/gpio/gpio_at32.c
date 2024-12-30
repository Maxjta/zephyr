/*
 * Copyright (c) 2022 YuLong Yao <feilongphone@gmail.com>
 * Copyright (c) 2021 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT at_at32_gpio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/interrupt_controller/intc_at32.h>
#include <zephyr/drivers/reset.h>

#include <soc.h>

/** SYSCFG DT node */
#define SYSCFG_NODE DT_NODELABEL(syscfg)

/** EXINT mask */
#define EXINT_MSK	       0xFU
/** EXINT line step size */
#define EXINT_STEP	       4U
/** EXINT line shift */
#define EXINT_LINE_SHIFT(pin) (EXINT_STEP * ((pin) % EXINT_STEP))

/* registers definitions */
#define GPIO_CFGR(gpiox)  REG32((gpiox) + 0x00000000U) /*!< GPIO port configuration register */
#define GPIO_OMODE(gpiox) REG32((gpiox) + 0x00000004U) /*!< GPIO port output mode register */
#define GPIO_ODRVR(gpiox) REG32((gpiox) + 0x00000008U) /*!< GPIO port drive capability register */
#define GPIO_PULL(gpiox)  REG32((gpiox) + 0x0000000CU) /*!< GPIO port pull-up/pull-down register */
#define GPIO_IDT(gpiox)   REG32((gpiox) + 0x00000010U) /*!< GPIO port input data register */
#define GPIO_ODT(gpiox)   REG32((gpiox) + 0x00000014U) /*!< GPIO port output data register */
#define GPIO_SCR(gpiox)	  REG32((gpiox) + 0x00000018U) /*!< GPIO port set/clear register */
#define GPIO_WPR(gpiox)   REG32((gpiox) + 0x0000001CU) /*!< GPIO port write protection register */
#define GPIO_MUXL(gpiox)  REG32((gpiox) + 0x00000020U) /*!< GPIO multiplexed function low register */
#define GPIO_MUXH(gpiox)  REG32((gpiox) + 0x00000024U) /*!< GPIO multiplexed function high register */
#define GPIO_CLR(gpiox)	  REG32((gpiox) + 0x00000028U) /*!< GPIO bit clear register */
#define GPIO_TOGR(gpiox)  REG32((gpiox) + 0x0000002CU) /*!< GPIO port bit toggle register */
#define GPIO_HDRV(gpiox)  REG32((gpiox) + 0x0000003CU) /*!< GPIO port huge current control register */

/* GPIO mode configuration values */
#define GPIO_MODE_SET(n, mode) ((uint32_t)((uint32_t)(mode) << (2U * (n))))
#define GPIO_MODE_MASK(n)      ((uint32_t)((uint32_t)0x00000003U << (2U * (n))))
#define GPIO_PIN_OFFSET(n)     (1 << n)

/* GPIO pull-up/pull-down values */
#define GPIO_PUPD_SET(n, pupd) ((uint32_t)((uint32_t)(pupd) << (2U * (n))))
#define GPIO_PUPD_MASK(n)      ((uint32_t)((uint32_t)0x00000003U << (2U * (n))))

struct gpio_at32_config {
	struct gpio_driver_config common;
	uint32_t reg;
	uint32_t rcu_periph_clock;
	uint32_t clkid;
	uint32_t clkid_exti;
	struct reset_dt_spec reset;
};

struct gpio_at32_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

/**
 * @brief EXINT ISR callback.
 *
 * @param line EXINT line (equals to GPIO pin number).
 * @param arg GPIO port instance.
 */
static void gpio_at32_isr(uint32_t line, void *arg)
{
	const struct device *dev = arg;
	struct gpio_at32_data *data = dev->data;

	gpio_fire_callbacks(&data->callbacks, dev, line);
}

/**
 * @brief Configure EXINT source selection register.
 *
 * @param port GPIO port instance.
 * @param pin GPIO pin number.
 *
 * @retval 0 on success.
 * @retval -EINVAL if pin is not valid.
 */
static int gpio_at32_configure_extiss(const struct device *port, gpio_pin_t pin)
{
	const struct gpio_at32_config *config = port->config;
	at32_exint_set_line_src_port(pin, config->reg);
	return 0;
}

static inline int gpio_at32_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_at32_config *config = port->config;
	gpio_type *gpio = (gpio_type *)config->reg;
	gpio_init_type init;
	uint32_t ctl, pupd;
	
	gpio_default_para_init(&init);
	
	init.gpio_pins = GPIO_PIN_OFFSET(pin);

	ctl = gpio->cfgr;//GPIO_CFGR(config->reg);
	ctl &= ~GPIO_MODE_MASK(pin);

	pupd = gpio->pull;//GPIO_PULL(config->reg);
	pupd &= ~GPIO_PUPD_MASK(pin);

	if ((flags & GPIO_OUTPUT) != 0U) {
      init.gpio_mode = GPIO_MODE_OUTPUT;
	  if ((flags & GPIO_SINGLE_ENDED) != 0U) {
		  if ((flags & GPIO_LINE_OPEN_DRAIN) != 0U) {
				init.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
			} else {
				return -ENOTSUP;
	    }
	  } else {
		  init.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	  }
	} else if ((flags & GPIO_INPUT) != 0U) {
		init.gpio_mode = GPIO_MODE_INPUT;
	} else {
		init.gpio_mode = GPIO_MODE_ANALOG;
	}

	if ((flags & GPIO_PULL_UP) != 0U) {
		init.gpio_pull = GPIO_PULL_UP_1;
	} else if ((flags & GPIO_PULL_DOWN) != 0U) {
		init.gpio_pull = GPIO_PULL_DOWN_1;
	} else {
		init.gpio_pull = GPIO_PULL_NONE;
	}
    
	gpio_init(gpio, &init);
	return 0;
}

static int gpio_at32_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_at32_config *config = port->config;
	gpio_type *gpio = (gpio_type *)config->reg;
	*value = gpio->idt;
	return 0;
}

static int gpio_at32_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_at32_config *config = port->config;
	gpio_type *gpio = (gpio_type *)config->reg;
	gpio->odt = (gpio->odt & ~mask) | (value & mask);
	return 0;
}

static int gpio_at32_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_at32_config *config = port->config;
  gpio_type *gpio = (gpio_type *)config->reg;
	gpio->scr = pins;
	return 0;
}

static int gpio_at32_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_at32_config *config = port->config;
	gpio_type *gpio = (gpio_type *)config->reg;
	gpio->clr = pins;
	return 0;
}

static int gpio_at32_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_at32_config *config = port->config;
	gpio_type *gpio = (gpio_type *)config->reg;

#ifdef CONFIG_AT32_HAS_AF_PINMUX
	gpio->togr = pins;
#else
  gpio->odt ^= pins;
#endif /* CONFIG_AT32_HAS_AF_PINMUX */
	return 0;
}

static int gpio_at32_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	if (mode == GPIO_INT_MODE_DISABLED) {
		at32_exint_intc_disable_line(BIT(pin));
		at32_exint_intc_select_line_trigger(BIT(pin), AT32_GPIO_IRQ_TRIG_NONE);
		at32_exint_intc_remove_irq_callback(BIT(pin));
	} else if (mode == GPIO_INT_MODE_EDGE) {
		int ret;
		at32_exint_intc_set_irq_callback(BIT(pin), gpio_at32_isr, (void *)port);
		ret = gpio_at32_configure_extiss(port, pin);
		if (ret < 0) {
			return ret;
		}

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			at32_exint_intc_select_line_trigger(BIT(pin), AT32_GPIO_IRQ_TRIG_FALLING);
			break;
		case GPIO_INT_TRIG_HIGH:
			at32_exint_intc_select_line_trigger(BIT(pin), AT32_GPIO_IRQ_TRIG_RISING);
			break;
		case GPIO_INT_TRIG_BOTH:
			at32_exint_intc_select_line_trigger(BIT(pin), AT32_GPIO_IRQ_TRIG_BOTH);
			break;
		default:
			return -ENOTSUP;
		}
    at32_exint_intc_enable_line(BIT(pin));
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static int gpio_at32_manage_callback(const struct device *dev, struct gpio_callback *callback,
				     bool set)
{
	struct gpio_at32_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static const struct gpio_driver_api gpio_at32_api = {
	.pin_configure = gpio_at32_configure,
	.port_get_raw = gpio_at32_port_get_raw,
	.port_set_masked_raw = gpio_at32_port_set_masked_raw,
	.port_set_bits_raw = gpio_at32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_at32_port_clear_bits_raw,
	.port_toggle_bits = gpio_at32_port_toggle_bits,
	.pin_interrupt_configure = gpio_at32_pin_interrupt_configure,
	.manage_callback = gpio_at32_manage_callback,
};

static int gpio_at32_init(const struct device *port)
{
	const struct gpio_at32_config *config = port->config;
	
#if 1
	(void)clock_control_on(AT32_CLOCK_CONTROLLER, (clock_control_subsys_t *)&config->clkid);

	/* enable access to SYSCFG_EXTISSn registers */
	(void)clock_control_on(AT32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t *)&config->clkid_exti);
#endif
	return 0;
}

#define GPIO_AT32_DEFINE(n)                                                                        \
	static const struct gpio_at32_config gpio_at32_config##n = {                               \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.clkid = DT_INST_CLOCKS_CELL(n, id),                                               \
		.clkid_exti = DT_CLOCKS_CELL(SYSCFG_NODE, id),                                     \
	};                                                                                         \
                                                                                                   \
	static struct gpio_at32_data gpio_at32_data##n;                                            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &gpio_at32_init, NULL, &gpio_at32_data##n, &gpio_at32_config##n,  \
			      PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_at32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_AT32_DEFINE)
