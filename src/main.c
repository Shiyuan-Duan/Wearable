#include <zephyr/kernel.h>

int main(void)
{       
        int counter = 0;
        // printk("This is from DK!\n");
        while(1)
        {
                printk("Counter: %d\n", counter);
                counter++;
                k_sleep(K_MSEC(3000));
        }
        return 0;
}
