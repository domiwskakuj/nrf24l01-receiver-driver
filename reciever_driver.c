#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define major(x)        ((int32_t)(((u_int32_t)(x) >> 24) & 0xff))
#define minor(x)        ((int32_t)((x) & 0xffffff))

#define IS_SETTED(bit, mask) (mask & bit)
#define N_SPI_MINORS 9
#define MODULE_NAME "nrf24"

/*registers*/
#define CONFIG_REG      0x00
#define EN_AA_REG       0x01
#define EN_RXADDR_REG   0x02
#define SETUP_AW_REG    0x03
#define SETUP_RETR_REG  0x04
#define RF_CH_REG       0x05
#define RF_SETUP_REG    0x06
#define STATUS_REG      0x07
#define OBSERVE_TX_REG  0x08
#define CD_REG          0x09
#define RX_ADDR_P0_REG  0x0a
#define RX_ADDR_P1_REG  0x0b
#define RX_ADDR_P2_REG  0x0c
#define RX_ADDR_P3_REG  0x0d
#define RX_ADDR_P4_REG  0x0e
#define RX_ADDR_P5_REG  0x0f
#define TX_ADDR_REG     0x10
#define RX_PW_P0_REG    0x11
#define RX_PW_P1_REG    0x12
#define RX_PW_P2_REG    0x13
#define RX_PW_P3_REG    0x14
#define RX_PW_P4_REG    0x15
#define RX_PW_P5_REG    0x16
#define FIFO_STATUS_REG 0x17
#define DYNPD_REG       0x1c
#define FEATURE_REG     0x1d

/*commands*/
#define R_REGISTER(addr) (addr)
#define W_REGISTER(addr) ((addr) + 0x20)
#define W_ACK_PAYLOAD(p) (0xa8 | (0x07 & p))  /* write ack payload */
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xa0
#define FLUSH_TX     0xe1
#define FLUSH_RX     0xe2
#define REUSE_TX_PL  0xe3
#define ACTIVATE     0x50
#define R_RX_PL_WID  0x60
#define W_TX_PAYLOAD_NO_ACK 0xb0
#define NOP_CMD      0xff


int write_reg(struct spi_device *spi, u8 reg, u64 val, int len);
int read_reg(struct spi_device *spi, u8 reg, u8 *val, int len);
void ce_high(struct spi_device *spi);
void ce_low(struct spi_device *spi);
void pwr_up(struct spi_device *spi);
void pwr_down(struct spi_device *spi);
void prx_mode(struct spi_device *spi);
int flush_rx(struct spi_device *spi);
int read_rx(struct spi_device *spi, u8 *payload, int len);
void enable_irqs(struct spi_device *spi);
void disable_irqs(struct spi_device *spi);

struct rf_data {
        dev_t devt;
        struct spi_device *spi;
        struct list_head device_entry;
        struct mutex bus_lock;
        unsigned int gpio_ce;
#define DEVNAME_MAX 16
        char devname[DEVNAME_MAX];
        struct completion tx_complete;
        wait_queue_head_t rx_wq;
        int new_rx_data;
        /* statistics */
        unsigned long txcount;
        unsigned long rxcount;
        unsigned long tx_kbps;
        unsigned long rx_kbps;
        struct task_struct *statistics_tsk;
};

static struct class *rf_class;
static struct cdev   rf_cdev;
static        dev_t  rf_devt;
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);



int write_reg(struct spi_device *spi, u8 reg, u64 val, int len)
{
        int status;
        u8 cmd = W_REGISTER(reg);

        struct spi_message m;
        struct spi_transfer t[2];

        if(len == 1)
        {
                u8 vall = val & 0xff;
                
                printk("%d\n", val);

                t[0].tx_buf = &cmd;
                t[0].len = 1;
                t[1].tx_buf = &vall;
                t[1].len = len;
        }
        else
        {
                u64 vall = val & 0x000000ffffffffff;

                printk("%d\n", val);

                t[0].tx_buf = &cmd;
                t[0].len = 1;
                t[1].tx_buf = &vall;
                t[1].len = len;
        }

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while wrinting on SPI");
        return status;
}

int read_reg(struct spi_device *spi, u8 reg, u8 *val, int len)
{
        int status;
        u8 cmd = R_REGISTER(reg);
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = val,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading from SPI");
        return status;
}

void ce_high(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);
        gpio_set_value(rf->gpio_ce, 1);
}

void ce_low(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);
        gpio_set_value(rf->gpio_ce, 0);
}

void pwr_up(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        if (IS_SETTED(BIT(1), config))
                return;
        config |= BIT(1);       /* set PWR_UP */
        write_reg(spi, CONFIG_REG, config, 1);
}

void pwr_down(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        if (!IS_SETTED(BIT(1), config))
                return;
        config &= ~(BIT(1));    /* clear PWR_UP */
        write_reg(spi, CONFIG_REG, config, 1);
}

void enable_irqs(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        config &= ~(BIT(4) | BIT(5) | BIT(6));
        write_reg(spi, CONFIG_REG, config, 1);
}

void disable_irqs(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        config |= (BIT(4) | BIT(5) | BIT(6));
        write_reg(spi, CONFIG_REG, config, 1);
}

void fix_config(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        config |= BIT(7);
        write_reg(spi, CONFIG_REG, config, 1);
}

void prx_mode(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        config |= BIT(0);
        write_reg(spi, CONFIG_REG, config, 1);
        printk("%d\n", config);
}

int flush_rx(struct spi_device *spi) // wyczysc kolejke RX FIFO
{
        int status;
        u8 cmd = FLUSH_RX;

        struct spi_message m;
        struct spi_transfer t = {
                .tx_buf = &cmd,
                .len = 1,
        };


        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while sending FLUSH_RX command on SPI");
        
        return status;
}

int read_rx(struct spi_device *spi, u8 *payload, int len)
{
        int status;
        u8 cmd = R_RX_PAYLOAD;
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = payload,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading RX payload from SPI");

        return status;
}


ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct spi_device *spi = to_spi_device(dev);
        int off = 0;
        u8 reg;
        u8 regval[5];

        for (reg = CONFIG_REG; reg <= FIFO_STATUS_REG; reg++) {
               switch (reg) {
                case CONFIG_REG:
                case EN_AA_REG:
                case EN_RXADDR_REG:
                case SETUP_AW_REG :
                case SETUP_RETR_REG:
                case RF_CH_REG:
                case RF_SETUP_REG:
                case STATUS_REG:
                case OBSERVE_TX_REG:
                case CD_REG:
                case RX_PW_P0_REG:
                case RX_PW_P1_REG:
                case RX_PW_P2_REG:
                case RX_PW_P3_REG:
                case RX_PW_P4_REG:
                case RX_PW_P5_REG:
                case FIFO_STATUS_REG:
                case RX_ADDR_P2_REG:
                case RX_ADDR_P3_REG:
                case RX_ADDR_P4_REG:
                case RX_ADDR_P5_REG:
                        read_reg(spi, reg, regval, 1);
                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x\n", reg, regval[0]);
                        break;
                case TX_ADDR_REG:
                case RX_ADDR_P0_REG:
                case RX_ADDR_P1_REG:
                        do {
                                u8 aw;

                                read_reg(spi, SETUP_AW_REG, &aw, 1);
                                aw +=  2;
                                read_reg(spi, reg, regval, 5);
                                switch (aw) {
                                case 3:
                                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x%02x%02x\n", reg, 
                                                         regval[0], regval[1], regval[2]);
                                        break;
                                case 4:
                                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x%02x%02x%02x\n", reg, 
                                                         regval[0], regval[1], regval[2], regval[3]);
                                        break;
                                case 5:
                                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x%02x%02x%02x%02x\n", reg, 
                                                         regval[0], regval[1], regval[2], regval[3], regval[4]);
                                        break;

                                }
                        } while (0);
                        break;
                default:
                        printk("Unknwon register addr %02x\n", reg);
                        break;
                }
        }

        for (reg = DYNPD_REG; reg <= FEATURE_REG; reg++) { 
                read_reg(spi, reg, regval, 1);
                off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x\n", reg, regval[0]);
        }

        return off;
}

ssize_t show_statistics(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct spi_device *spi = to_spi_device(dev);
        struct rf_data *rf     = spi_get_drvdata(spi);

        return snprintf(buf, PAGE_SIZE,
                        "TX kbps: %5lu\n"
                        "RX kbps: %5lu\n"
                        "TX count: %5lu\n"
                        "RX count: %5lu\n",
                        rf->tx_kbps,
                        rf->rx_kbps,
                        rf->txcount,
                        rf->rxcount);

}

ssize_t show_config(struct device *dev, struct device_attribute *attr, char *buf)
{
        u8 regval;
        unsigned off = 0;
        struct spi_device *spi = to_spi_device(dev);

        read_reg(spi, RF_SETUP_REG, &regval, 1);
        if (IS_SETTED(BIT(5), regval))
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Data rate:     250kbps\n");
        else if (IS_SETTED(BIT(3), regval))
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Data rate:     2mbps\n");
        else
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Data rate:     1mbps\n");

        switch ((regval & 0x6) >> 1) { /* get bit 1 and 2 */
        case 0:                        /* 00 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   -18dBm\n");
                break;
        case 1:                 /* 01 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   -12dBm\n");
                break;
        case 2:                 /* 10 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   -6dBm\n");
                break;
        case 3:                 /* 11 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   0dBm\n");
                break;
        }

        read_reg(spi, RF_CH_REG, &regval, 1);
        off += snprintf(buf + off, PAGE_SIZE - off,
                        "Channel:       %u\n", regval);

        read_reg(spi, SETUP_AW_REG, &regval, 1);
        off += snprintf(buf + off, PAGE_SIZE - off,
                        "Address width: %u\n", regval + 2);

        read_reg(spi, FEATURE_REG, &regval, 1);
        off += snprintf(buf + off, PAGE_SIZE - off,
                        "EN_DPL:        %u\n"
                        "EN_ACK_PAY:    %u\n"
                        "EN_DYN_ACK:    %u\n",
                        (IS_SETTED(BIT(2), regval) ? 1 : 0),
                        (IS_SETTED(BIT(1), regval) ? 1 : 0),
                        (IS_SETTED(BIT(0), regval) ? 1 : 0));


        return off;
}

int available(struct spi_device* spi)
{
    u8 regval;
    read_reg(spi, CONFIG_REG, &regval, 1);
    printk("CONFIG %d\n", regval);
    read_reg(spi, FIFO_STATUS_REG, &regval, 1);
    printk("FIFO_STATUS %d\n", regval);
    
    
    if ((regval & 0x01) == 0x00) // there is something in RX FIFO
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void check_RX_setup(struct spi_device* spi)
{
    u8 regval;
    printk("nrf24 - checking RX setup\n");

    read_reg(spi, SETUP_AW_REG, &regval, 1);
    printk("SETUP_AW %d\n", regval);
    read_reg(spi, RX_ADDR_P0_REG, &regval, 1);
    printk("RX_ADDR_P0 %d\n", regval);
    read_reg(spi, CONFIG_REG, &regval, 1);
    printk("CONFIG %d\n", regval);
    read_reg(spi, EN_RXADDR_REG, &regval, 1);
    printk("EN_RXADDR %d\n", regval);
    read_reg(spi, RF_CH_REG, &regval, 1);
    printk("RF_CH %d\n", regval);
    read_reg(spi, RF_SETUP_REG, &regval, 1);
    printk("RF_SETUP %d\n", regval);
    read_reg(spi, STATUS_REG, &regval, 1);
    printk("STATUS %d\n", regval);
}
/*
 * Sysfs attributes
 */
static DEVICE_ATTR(registers, 0600, show_reg, NULL);
static DEVICE_ATTR(statistics, 0400, show_statistics, NULL);
static DEVICE_ATTR(config, 0400, show_config, NULL);

static struct attribute *rf_attrs[] = {
        &dev_attr_registers.attr,
        &dev_attr_statistics.attr,
        &dev_attr_config.attr,
        NULL,
};

static struct attribute_group rf_attr_group = {
        .attrs = rf_attrs,
};

static ssize_t rf_read(struct file *fp, char *buf, size_t len, loff_t *off)
{
        char pld[32];
        int n;
        int rcvd = 32;
        int status;
        struct rf_data *rf = fp->private_data;

        if (!rf)
                return -ENODEV;


        //RX(rf, pld, rcvd, status);
        rf->new_rx_data = 0;                                  
        //ce_high(spi);                                     
        status = available(rf->spi);                          
        //ce_low(spi);                                      
        if (status > 0) {                                       
                printk("Starting: reading\n");                  
                read_rx(rf->spi, pld, 32);                
                printk("Payload: %c\n", pld[0]);                                          
        }  
        if (status < 0) {
                printk("RX ERROR: %d\n", status);
                return status;
        }
        if (status == 0)
        {
            printk("RX FIFO empty\n");
        }
        //check_RX_setup(rf->spi);
        n = copy_to_user(&buf[*off], pld, rcvd);
        if (n)
        {
                printk("cannot copy\n");
                return -EIO;
        }
                
        *off += rcvd;

        return rcvd;
}

static int rf_release(struct inode *ip, struct file *fp)
{
        struct rf_data *rf = fp->private_data;
        printk("release\n");
        if (!rf)
                return -ENODEV;

        
        fp->private_data = NULL;
        return 0;
}

static int rf_open(struct inode *ip, struct file *fp)
{
        struct rf_data *rf;

        if (fp->private_data)
                return 0;

        mutex_lock(&device_list_lock);
        list_for_each_entry(rf, &device_list, device_entry) {
                if (rf->devt == ip->i_rdev) {
                        fp->private_data = rf;
                        //SB1_MODE(rf->spi);
                        mutex_unlock(&device_list_lock);
                        udelay(150);                             /* power up delay */
                        return 0;
                }
        }
        mutex_unlock(&device_list_lock);
        return -ENODEV;
}


static struct file_operations fops = {
        .owner = THIS_MODULE,
        .read = rf_read,
        .open = rf_open,
        .release = rf_release,
};

/*
 * Driver operations
 */
static int rf_probe(struct spi_device *spi)
{
        printk("nrf24 - starting probe\n");
        int status;
        unsigned long minor;
        struct device *devp;
        struct rf_data *rf;
        struct device_node *np = spi->dev.of_node;
        int gpio_irq;

        if (!np)
        {
            printk("nrf24 - np is NULL\n");
            return -ENODEV;
        }
        rf = kzalloc(sizeof (struct rf_data), GFP_KERNEL);
        if (!rf)
        {
            printk("nrf24 - kzalloc failed\n");
            return -ENOMEM;
        }
        rf->spi = spi;
        rf->new_rx_data = 0;
        rf->gpio_ce = of_get_named_gpio(np, "gpio-ce", 0);
        if (rf->gpio_ce < 0)
        {
            printk("nrf24 - gpio-ce failure\n");
            return -ENODEV;
        }
                
        INIT_LIST_HEAD(&rf->device_entry);
        mutex_init(&rf->bus_lock);
        init_completion(&rf->tx_complete);
        init_waitqueue_head(&rf->rx_wq);
        scnprintf(rf->devname, DEVNAME_MAX, "nrf24");
        //rf->devname = "nrf24";

        status = spi_setup(spi);
        if (status)
        {
            printk("nrf24 - spi setup failure\n");
            kfree(rf);
            return status;
        }
        spi_set_drvdata(spi, rf);
        status = gpio_request(rf->gpio_ce, rf->devname);
        if (status)
        {
            printk("nrf24 - gpio request failure, status - %d\n", status);
            kfree(rf);
            return status;
        }
        gpio_direction_output(rf->gpio_ce, 0);

        /* setup */
        printk("nrf24 - starting setup\n");
        write_reg(spi, RX_PW_P0_REG, 0x20, 1); /* setup 32bytes of payload by default */
        write_reg(spi, STATUS_REG, 0x70, 1); /* Reset status register */
        write_reg(spi, RF_SETUP_REG, 0x00, 1); /* set speed to 1Mbps */
        write_reg(spi, EN_RXADDR_REG, 0x01, 1); /* enable only data pipe 0 */
        write_reg(spi, RF_CH_REG, 0x64, 1); /* set channel to 100*/
        write_reg(spi, SETUP_AW_REG, 0x03, 1); /* setting RX address field width*/
        write_reg(spi, RX_ADDR_P0_REG, 0x7878787878, 5);
        disable_irqs(spi);                        /* disable irqs (are enabled on transmitting/receiving */


        check_RX_setup(spi);

        printk("nrf24 - setup complete\n");

        /* Sysfs attributes */
        status = sysfs_create_group(&spi->dev.kobj, &rf_attr_group);
        if (status){
            printk("nrf24 - sysfs group creation failure, status - %d\n", status);
            gpio_free(rf->gpio_ce);
            return status;
        }

        /* Device creation */
        mutex_lock(&device_list_lock);
        minor = find_first_zero_bit(minors, N_SPI_MINORS);
        if (minor > N_SPI_MINORS) {
            mutex_unlock(&device_list_lock);
            status = -ENODEV;
            printk("nrf24 - getting minor number failure, status - %d\n", status);
            return status;
        }

        rf->devt = MKDEV(MAJOR(rf_devt), MINOR(minor));
        devp = device_create(rf_class, &spi->dev, rf->devt, rf, "%s", rf->devname);
        if (IS_ERR(devp)) {
            mutex_unlock(&device_list_lock);
            status = PTR_ERR(devp);
            printk("nrf24 - device creation failure, status - %d\n", status);
            return status;
        }
        set_bit(minor, minors);
        list_add(&rf->device_entry, &device_list);
        mutex_unlock(&device_list_lock);

	printk("nrf24 - probe done\n");
        check_RX_setup(spi);

        printk("Change to STANDBY-I MODE\n"); 
        pwr_up(spi);                          
        ce_low(spi); 
        u8 regval;
        udelay(150);
        prx_mode(spi); 
        read_reg(spi, CONFIG_REG, &regval, 1);
        printk("CONFIG %d\n", regval);
        ce_high(spi);
        
        flush_rx(spi);  
        write_reg(spi, SETUP_AW_REG, 0x03, 1);                               
        write_reg(spi, STATUS_REG, 0x70, 1);                       
        write_reg(spi, RF_CH_REG, 0x64, 1);   
        write_reg(spi, EN_AA_REG, 0x01, 1);          
        write_reg(spi, RX_ADDR_P0_REG, 0x7878787878, 5); 
        prx_mode(spi); 
        ce_high(spi);  

        check_RX_setup(spi);
        return 0;
}

static void rf_remove(struct spi_device *spi)
{
        ce_low(spi);
        pwr_down(spi);

        struct rf_data *rf = spi_get_drvdata(spi);
        
        mutex_lock(&device_list_lock);
        list_del(&rf->device_entry);
        clear_bit(MINOR(rf->devt), minors);
        device_destroy(rf_class, rf->devt);
        spi_set_drvdata(spi, NULL);
        mutex_unlock(&device_list_lock);

        gpio_free(rf->gpio_ce);

        sysfs_remove_group(&spi->dev.kobj, &rf_attr_group);

        kfree(rf);
	    printk("nrf24 - remove done\n");
}

static const struct of_device_id rf_of_ids[] = {
        { .compatible = "domi,nrf24" },
        {},
};

static struct spi_driver rf_driver = {
        .driver = {
                .name  = "nrf24",
                .owner = THIS_MODULE,
                .of_match_table = rf_of_ids,
        },
        .probe  = rf_probe,
        .remove = rf_remove,
};

static int __init rf_init(void)
{
        int status;

        status = alloc_chrdev_region(&rf_devt, 0, N_SPI_MINORS, "nrf24");
        if (status<0)
	    {
		    printk("nrf24 - allocating character device failed, status: %d\n", status);
            return status;
	    }
        cdev_init(&rf_cdev, &fops);
        status = cdev_add(&rf_cdev, rf_devt, N_SPI_MINORS);
        if (status < 0)  {
		    printk("nrf24 - adding character device failed, status: %d\n", status);
            unregister_chrdev_region(rf_devt, N_SPI_MINORS);
            return status;
        }
        rf_class = class_create("nrf24");
        if (IS_ERR(rf_class)) {
		        printk("nrf24 - class creation failed, status: %d\n", status);
                status = PTR_ERR(rf_class);
                cdev_del(&rf_cdev);
                return status;
        }
        status = spi_register_driver(&rf_driver);
        if (status)
	    {
		    printk("nrf24 - driver registration failed, status: %d\n", status);
            class_destroy(rf_class);
            return status;
	    }
	    printk("nrf24 - hello kernel\n");
        return 0;
}

static void __exit rf_exit(void)
{
        spi_unregister_driver(&rf_driver);
        class_destroy(rf_class);
        cdev_del(&rf_cdev);
        unregister_chrdev_region(rf_devt, N_SPI_MINORS);
	printk("nrf24 - goodbye kernel\n\n");
}

module_init(rf_init);
module_exit(rf_exit);
MODULE_AUTHOR("domiwskakuj");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("nRF24L01+ driver for receiving data");