
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/i2c.h>

#include "i2c.h"

#define pr_dbg2(args...) printk(KERN_DEBUG "OpenVFD: " args)
#define LOW	0
#define HIGH	1

static unsigned char i2c_read_cmd_data(const unsigned char *cmd, unsigned int cmd_length, unsigned char *data, unsigned int data_length);
static unsigned char i2c_read_data(unsigned char *data, unsigned int length);
static unsigned char i2c_read_byte(unsigned char *bdata);
static unsigned char i2c_write_cmd_data(const unsigned char *cmd, unsigned int cmd_length, const unsigned char *data, unsigned int data_length);
static unsigned char i2c_write_data(const unsigned char *data, unsigned int length);
static unsigned char i2c_write_byte(unsigned char bdata);

static struct protocol_interface i2c_interface = {
	.read_cmd_data = i2c_read_cmd_data,
	.read_data = i2c_read_data,
	.read_byte = i2c_read_byte,
	.write_cmd_data = i2c_write_cmd_data,
	.write_data = i2c_write_data,
	.write_byte = i2c_write_byte,
	.protocol_type = PROTOCOL_TYPE_I2C
};

union address {
	struct {
		unsigned char low;
		unsigned char high;
	} nibbles;
	unsigned short value;
};

static void i2c_stop_condition(void);

static union address i2c_address = { 0 };
static struct i2c_adapter *i2c;
static unsigned char use_address = 0;
static unsigned char long_address = 0;
static unsigned long i2c_delay = I2C_DELAY_100KHz;
static unsigned char lsb_first = 0;
static unsigned short clk_stretch_timeout = 0;
static struct vfd_pin pin_scl = { 0 };
static struct vfd_pin pin_sda = { 0 };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
inline void gpio_set_pullup(unsigned gpio, int value)
{
	gpio_direction_input(gpio);
	gpio_set_value(gpio, value);
}
#endif

static unsigned char i2c_test_connection(void)
{
	return i2c_write_cmd_data(NULL, 0, NULL, 0);
}

struct protocol_interface *init_i2c(unsigned short _address, unsigned char _lsb_first, struct vfd_pin _pin_scl, struct vfd_pin _pin_sda, unsigned long _i2c_delay)
{
	struct protocol_interface *i2c_ptr = NULL;
	if (_pin_scl.pin >= 0 && _pin_sda.pin >= 0) {
		if (_address) {
			use_address = 1;
			long_address = _address > 0xFF;					// A valid 10-bit address always starts with b11110.
			i2c_address.value = _address == 0xFF ? 0x0000 : _address;	// General call.
		} else {
			use_address = 0;
		}
		i2c = i2c_get_adapter(3); // FIXME only for test fixed to i2c_3, Odroid N2 pin 27,28
		if (i2c)
			pr_dbg2("Found i2c-%d adapter: %s\n", 3 , i2c->name);

		if (!i2c_test_connection()) {
			i2c_ptr = &i2c_interface;
			pr_dbg2("I2C interface intialized (address = 0x%04X%s, %s mode, pull-ups %s)\n", i2c_address.value, !use_address ? " (N/A)" : "",
				lsb_first ? "LSB" : "MSB", _pin_scl.flags.bits.pullup_on ? "on" : "off" );
		} else {
			pr_dbg2("I2C interface failed to intialize. Could not establish communication with I2C slave\n");
		}
	} else {
		pr_dbg2("I2C interface failed to intialize. Invalid SCL (%d) and/or SDA (%d) pins\n", _pin_scl.pin, _pin_sda.pin);
	}
	return i2c_ptr;
}

static inline void gpio_set_pin_low(const struct vfd_pin *pin)
{
	gpio_direction_output(pin->pin, LOW);
}

static inline void gpio_set_pin_high(const struct vfd_pin *pin)
{
	if (pin->flags.bits.kick_high)
		gpio_direction_output(pin->pin, HIGH);
	gpio_direction_input(pin->pin);
}

static void i2c_start_condition(void)
{
	gpio_set_pin_low(&pin_sda);
	udelay(i2c_delay);
	gpio_set_pin_low(&pin_scl);
	udelay(i2c_delay);
}

static void i2c_stop_condition(void)
{
	gpio_set_pin_high(&pin_scl);
	udelay(i2c_delay);
	gpio_set_pin_high(&pin_sda);
	udelay(i2c_delay);
	udelay(i2c_delay);
}

static inline unsigned char i2c_ack(void)
{
	unsigned char ret = 1, scl = 1;
	unsigned short timeout = clk_stretch_timeout;
	gpio_set_pin_low(&pin_scl);
	gpio_set_pin_high(&pin_sda);
	udelay(i2c_delay);
	gpio_set_pin_high(&pin_scl);
	udelay(i2c_delay);
	do {
		scl = gpio_get_value(pin_scl.pin) ? 1 : 0;
		udelay(1);
	} while (!scl && timeout--);
	ret = gpio_get_value(pin_sda.pin) ? 1 : 0;
	gpio_set_pin_low(&pin_scl);
	gpio_set_pin_low(&pin_sda);
	udelay(i2c_delay);
	return ret;
}

static unsigned char i2c_write_raw_byte(unsigned char data)
{
	unsigned char i = 8;
	unsigned char mask = lsb_first ? 0x01 : 0x80;
	gpio_set_pin_low(&pin_scl);
	while (i--) {
		if (data & mask)
			gpio_set_pin_high(&pin_sda);
		else
			gpio_set_pin_low(&pin_sda);
		udelay(i2c_delay);
		gpio_set_pin_high(&pin_scl);
		udelay(i2c_delay);
		gpio_set_pin_low(&pin_scl);
		if (lsb_first)
			data >>= 1;
		else
			data <<= 1;
	}
	return i2c_ack();
}

static unsigned char i2c_read_raw_byte(unsigned char *data)
{
	unsigned char i = 8;
	unsigned char mask = lsb_first ? 0x80 : 0x01;
	*data = 0;
	gpio_set_pin_high(&pin_sda);
	while (i--) {
		if (lsb_first)
			*data >>= 1;
		else
			*data <<= 1;
		gpio_set_pin_high(&pin_scl);
		udelay(i2c_delay);
		if (gpio_get_value(pin_sda.pin))
			*data |= mask;
		gpio_set_pin_low(&pin_scl);
		udelay(i2c_delay);
	}
	return i2c_ack();
}

static unsigned char i2c_write_address(union address _address, unsigned char rw)
{
	unsigned char ret = 0;
	if (long_address) {
		_address.nibbles.high <<= 1;
		_address.nibbles.high |= rw ? 0x01 : 0x00;
		ret = i2c_write_raw_byte(_address.nibbles.high);
		if (!ret)
			ret = i2c_write_raw_byte(_address.nibbles.low);
	} else {
		_address.nibbles.low <<= 1;
		_address.nibbles.low |= rw ? 0x01 : 0x00;
		ret = i2c_write_raw_byte(_address.nibbles.low);
	}
	return ret;
}
static int i2c_writereg(u8 *data, u16 *size)
{
	int ret;
	struct i2c_msg msg[1] = {
			{
				.addr = i2c_address,
				.flags = 0,
				.buf = data,
				.len = *size,
			}
	};

	ret = i2c_transfer(i2c, msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&i2c->dev, "i2c wr failed=%d", ret);
		ret = -EREMOTEIO;
	}

	return ret;
}

static int i2c_readreg(u8 * data, u16 * size)
{
	int ret;
	struct i2c_msg msg[1] = {
			{
					.addr = i2c_address,
					.flags = I2C_M_RD,
					.buf = data,
					.len = *size,
			}
	};

	ret = i2c_transfer(i2c, msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&i2c->dev, "i2c rd failed=%d", ret);
		ret = -EREMOTEIO;
	}

	return ret;
}
static unsigned char i2c_read_cmd_data(const unsigned char *cmd, unsigned int cmd_length, unsigned char *data, unsigned int data_length)
{
	unsigned char status = 0;

	status = i2c_readreg(data, dat_length);
	return status;
}

static unsigned char i2c_read_data(unsigned char *data, unsigned int length)
{
	return i2c_read_cmd_data(NULL, 0, data, length);
}

static unsigned char i2c_read_byte(unsigned char *bdata)
{
	return i2c_read_cmd_data(NULL, 0, bdata, 1);
}

static unsigned char i2c_write_cmd_data(const unsigned char *cmd, unsigned int cmd_length, const unsigned char *data, unsigned int data_length)
{
	unsigned char status = 0;
	u8 buf[20];

	if (cmd)
		memcpy(cmd, buf, cmd_length);

	if (cmd)
		memcpy(data + cmd_length, buf, data_length);

	status = i2c_writereg(buf, cmd_length + data_length);

return status;
}

static unsigned char i2c_write_data(const unsigned char *data, unsigned int length)
{
	return i2c_write_cmd_data(NULL, 0, data, length);
}

static unsigned char i2c_write_byte(unsigned char bdata)
{
	return i2c_write_cmd_data(NULL, 0, &bdata, 1);
}
