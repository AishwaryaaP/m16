//Library for ATmega 16
#define LCD_DATA PORTB                //LCD data port
#define ctrl PORTA
#define en PA2                        // enable signal
#define rw PA1                       // read/write signal
#define rs PA0                    // register select signal

void LCD_cmd(unsigned char cmd);
void init_LCD(void);
void LCD_write(unsigned char data);


void init_LCD(void)
{
	LCD_cmd(0x38);                            // initialization of 16X2 LCD in 8bit mode
	_delay_ms(1);
	LCD_cmd(0x01);                                 // clear LCD
	_delay_ms(1);
	LCD_cmd(0x0E);                        // cursor ON
	_delay_ms(1);
	LCD_cmd(0x80);                     // �8 go to first line and �0 is for 0th position
	_delay_ms(1);
	return;
}

void LCD_cmd(unsigned char cmd)
{
	LCD_DATA=cmd;
	ctrl =(0<<rs)|(0<<rw)|(1<<en);
	_delay_ms(1);
	ctrl =(0<<rs)|(0<<rw)|(0<<en);
	_delay_ms(50);
	return;
}

void LCD_write(unsigned char data)
{
	LCD_DATA= data;
	ctrl = (1<<rs)|(0<<rw)|(1<<en);
	_delay_ms(1);
	ctrl = (1<<rs)|(0<<rw)|(0<<en);
	_delay_ms(50);
	return ;
}

void LCD_write_string(unsigned char *str)             //store address value of the string in pointer *str
{
	int i=0;
	while(str[i]!='\0')                               // loop will go on till the NULL character in the string
	{
		LCD_write(str[i]);                            // sending data on LCD byte by byte
		i++;
	}
	return;
}

void LCD_write_data(int d )             //store address value of the string in pointer *str
{
	char *s= itoa (d, *s, 10);
	int j=0;
	while(s[j]!='\0')                               // loop will go on till the NULL character in the string
	{
		LCD_write(s[j]);                            // sending data on LCD byte by byte
		j++;
	}
	return;
}
