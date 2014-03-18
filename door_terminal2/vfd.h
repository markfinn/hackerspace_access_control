#define printf(format, ...) printf_P (PSTR(format), ##__VA_ARGS__)
#define lprintf(format, ...) fprintf_P (stderr, PSTR(format), ##__VA_ARGS__)
#define lprintE(format, ...) lprintf ("E:" format "\n", ##__VA_ARGS__)
#define lprintW(format, ...) lprintf ("W:" format "\n", ##__VA_ARGS__)
#if DEBUGPRINT == VFD
#define printD(format, ...) do{lprintf ("D:" format "\n", ##__VA_ARGS__); printf ("D:" (format), ##__VA_ARGS__); }while(0)
#elif defined DEBUGPRINT
#define printD(format, ...) do{lprintf ("D:" format "\n", ##__VA_ARGS__); }while(0)
#else
#define printD(format, ...) do{ }while(0)
#endif



#define VFDCOMMAND_BACKSPACE 				"\x08"
#define VFDCOMMAND_FORWARD 					"\x09"
#define VFDCOMMAND_CRLF 						"\x0a"  //composited by putchar routine
#define VFDCOMMAND_HOME 						"\x0b"
#define VFDCOMMAND_CLEARHOME 				"\x0c"
#define VFDCOMMAND_CR 							"\x0d"
#define VFDCOMMAND_OVERWRITE(o) 		"\x1f" #o 	//1==overwrite, (0==?)
#define VFDCOMMAND_SCROLLMODE(m) 		"\x1f" #m 	//2==vert, 3==horiz, (0==none?)
#define VFDCOMMAND_HSCROLLSPEED(s) 	"\x1fs" #s 	//speed==0: by character, speed==1: T mS / 2dots,  speed<32: (n-1)*T mS / dot
#define VFDCOMMAND_INVERT(o) 				"\x1fr" #o 	//{0,1}
#define VFDCOMMAND_COMPOSITION(m) 	"\x1fw" #m 	//{0-3}-> normal, or, and, xor
#define VFDCOMMAND_BRIGHTNESS(b) 		"\x1fX" #b 	//1-8
#define VFDCOMMAND_WAIT(t) 					"\x1f(a\x01" #t //each t approx .5 S
#define VFDCOMMAND "\x1f(a@" //
#define VFDCOMMAND_INITIALIZE 			"\x1b@"
/*
}

void Noritake_VFD_GU7000::GU7000_cursorOn() {
    command(0x1f, 'C', 1);
}

void Noritake_VFD_GU7000::GU7000_cursorOff() {
    command(0x1f, 'C', 0);
}

void Noritake_VFD_GU7000::GU7000_screenSaver(ScreenSaver mode) {
    us_command('a', 0x40);
    command(mode);
}

void Noritake_VFD_GU7000::GU7000_setScreenBrightness(unsigned level) {
    if (level == 0)
        GU7000_displayOff();
    else if (level <= 100) {
        GU7000_displayOn();
        command(0x1f, 'X', (level*10 + 120)/125);
    }
}

void Noritake_VFD_GU7000::GU7000_setBacklightColor(uint8_t r, uint8_t g, uint8_t b) {
    #if NORITAKE_VFD_MODEL_CLASS==7040
        command(0x1f, 'L', 0x10);
        command(b & 0xf0);
        command(g & 0xf0);
        command(r & 0xf0);
    #endif
}

void Noritake_VFD_GU7000::GU7000_setBacklightColor(unsigned rgb) {
    GU7000_setBacklightColor((rgb>>8 & 0x0f)*16, (rgb>>4 & 0x0f)*16, (rgb & 0x0f)*16);
}




void Noritake_VFD_GU7000::GU7000_setCursor(unsigned x, unsigned y) {
    command(0x1f);
    command('$');
    command_xy(x, y);
}


void Noritake_VFD_GU7000::GU7000_useMultibyteChars(bool enable) {
    #if (NORITAKE_VFD_MODEL_CLASS-7000)/100==9 || (NORITAKE_VFD_MODEL_CLASS-7000)/100==1
        us_command('g', 0x02);
        command(enable);
    #endif
}

void Noritake_VFD_GU7000::GU7000_setMultibyteCharset(uint8_t code) {
    #if (NORITAKE_VFD_MODEL_CLASS-7000)/100==9 || (NORITAKE_VFD_MODEL_CLASS-7000)/100==1
        us_command('g', 0x0f);
        command(code);
    #endif
}

void Noritake_VFD_GU7000::GU7000_useCustomChars(bool enable) {
    command(0x1b, '%', enable);
}

static inline uint8_t getColumn(const uint8_t *src, int col) {
    uint8_t out = 0;
    for (int i=0; i<8; i++)
        if (src[i] & (1<<(4-col))) out += 1<<(7-i);
    return out;
}

void Noritake_VFD_GU7000::GU7000_defineCustomChar(uint8_t code, FontFormat format, const uint8_t *data) {
    command(0x1b, '&', 0x01);
    command(code);
    command(code);
    
    switch (format) {
    case CUUFormat:
        command(5);
        for (uint8_t i=0; i<5; i++)
            command(getColumn(data, i));
        break;
    
    case GU70005x7Format:
        command(5);
        print((const char*)data, 5);
        break;
    case GU70007x8Format:
        command(7);
        print((const char*)data, 7);
        break;
    }
}

void Noritake_VFD_GU7000::GU7000_deleteCustomChar(uint8_t code) {
    command(0x01b, '?', 0x01);
    command(code);
}

void Noritake_VFD_GU7000::GU7000_setAsciiVariant(AsciiVariant code) {
    command(0x1b, 'R', code);
}

void Noritake_VFD_GU7000::GU7000_setCharset(Charset code) {
    command(0x1b, 't', code);
}


void Noritake_VFD_GU7000::GU7000_scrollScreen(unsigned x, unsigned y, unsigned times, uint8_t speed) {
    unsigned pos = (x*NORITAKE_VFD_LINES)+(y/8);
    us_command('a', 0x10);
    command(pos);
    command(pos>>8);
    command(times);
    command(times>>8);
    command(speed);
}

void Noritake_VFD_GU7000::GU7000_blinkScreen() {
    us_command('a', 0x11);
    command(0);
    command(0);
    command(0);
    command(0);
}

void Noritake_VFD_GU7000::GU7000_blinkScreen(bool enable, bool reverse, uint8_t onTime, uint8_t offTime, uint8_t times) {
    us_command('a', 0x11);
    command(enable? (reverse? 2: 1): 0);
    command(onTime);
    command(offTime);
    command(times);
}

void Noritake_VFD_GU7000::GU7000_displayOff() {
    us_command('a', 0x40);
    command(0);
}

void Noritake_VFD_GU7000::GU7000_displayOn() {
    us_command('a', 0x40);
    command(0x01);
}


void Noritake_VFD_GU7000::GU7000_setFontStyle(bool proportional, bool evenSpacing) {
    us_command('g', 0x03);
    command(proportional*2 + evenSpacing);
}

void Noritake_VFD_GU7000::GU7000_setFontSize(uint8_t x, uint8_t y, bool tall) {
    if (x<=4 && y<=2) {        
        us_command('g', 0x40);
        command(x);
        command(y);
        #if (NORITAKE_VFD_MODEL_CLASS-7000)/100==9 || (NORITAKE_VFD_MODEL_CLASS-7000)/100==1
            us_command('g', 0x01);
            command(tall+1);
        #endif
    }
}

void Noritake_VFD_GU7000::GU7000_selectWindow(uint8_t window) {
    if (window <= 4)
        command(0x10 + window);
}

void Noritake_VFD_GU7000::GU7000_defineWindow(uint8_t window, unsigned x, unsigned y, unsigned width, unsigned height) {
    us_command('w', 0x02);
    command(window);
    command(0x01);
    command_xy(x, y);
    command_xy(width, height);
}

void Noritake_VFD_GU7000::GU7000_deleteWindow(uint8_t window) {
    us_command('w', 0x02);
    command(window);
    command(0);
    command_xy(0, 0);
    command_xy(0, 0);
}

void Noritake_VFD_GU7000::GU7000_joinScreens() {
    us_command('w', 0x10);
    command(0x01);
}

void Noritake_VFD_GU7000::GU7000_separateScreens() {
    us_command('w', 0x10);
    command(0);
}



void Noritake_VFD_GU7000::GU7000_drawImage(unsigned width, uint8_t height, const uint8_t *data) {
    if (height > NORITAKE_VFD_HEIGHT) return;
    us_command('f', 0x11);
    command_xy(width, height);
    command((uint8_t) 1);
    for (unsigned i = 0; i<(height/8)*width; i++)
        command(data[i]);
}

void Noritake_VFD_GU7000::GU7000_drawFROMImage(unsigned long address, uint8_t srcHeight, unsigned width, uint8_t height) {
    #if (NORITAKE_VFD_MODEL_CLASS-7000)/100==9 || (NORITAKE_VFD_MODEL_CLASS-7000)/100==1        
        if (height > NORITAKE_VFD_HEIGHT) return;
        us_command('f', 0x10);
        command(0x01);
        command(address);
        command(address>>8);
        command(address>>16);
        command(srcHeight/8);
        command((srcHeight/8)>>8);
        command_xy(width, height);
        command((uint8_t) 1);
    #endif
}

static unsigned min(unsigned x, unsigned y) { return x<y? x: y; }

void Noritake_VFD_GU7000::GU7000_fillRect(unsigned x0, unsigned y0, unsigned x1, unsigned y1, bool on) {
    x0 = min(NORITAKE_VFD_WIDTH, x0);
    x1 = min(NORITAKE_VFD_WIDTH, x1);
    y0 = min(NORITAKE_VFD_HEIGHT, y0);
    y1 = min(NORITAKE_VFD_HEIGHT, y1);
    if (y1<=y0 || x1<=x0) return;
    uint8_t bufw = 8, bufh = (y1-y0+7)/8*8;
    uint8_t *buf = (uint8_t*)alloca(bufh/8 * bufw);
    for (unsigned x = 0; x < x1-x0; x += bufw) {
        uint8_t part = (x + bufw < x1-x0)? bufw: (x1-x0) - x;
        memset(buf, 0, bufh/8 * bufw);
        if (on)
            for (uint8_t col = 0; col < part; col++) {
                for (uint8_t py = y0 % 8; py < y0 % 8 + min(y1-y0, 8); py++)
                    buf[col*bufh/8] |= 1 << (7-py);
                for (uint8_t row = (y0+7)/8; row < y1/8; row++)
                    buf[row - y0/8 + col*bufh/8] = 0xff;
                if (y0/8 != y1/8)
                    for (uint8_t py = 0; py < y1 % 8; py++)
                        buf[(y1-y0)/8 + col*bufh/8] |= 1 << (7-py);
            }
        GU7000_setCursor(x + x0, y0);
        GU7000_drawImage(bufw, bufh, buf);
    }
}

void Noritake_VFD_GU7000::command(uint8_t data) {
    writePort(data);
}
void Noritake_VFD_GU7000::command_xy(unsigned x, unsigned y) {
    command(x);
    command(x>>8);
    y /= 8;
    command(y);
    command(y>>8);
}
void Noritake_VFD_GU7000::command_xy1(unsigned x, unsigned y) {
    command(x);
    command(x>>8);
    command(y);
    command(y>>8);
}

void Noritake_VFD_GU7000::us_command(uint8_t group, uint8_t cmd) {
   command(0x1f);
   command(0x28);
   command(group);
   command(cmd);
}

void Noritake_VFD_GU7000::command(uint8_t prefix, uint8_t group, uint8_t cmd) {
   command(prefix);
   command(group);
   command(cmd);
}

void Noritake_VFD_GU7000::print(unsigned x, uint8_t y, const char *buffer, uint8_t len) {
    #if NORITAKE_VFD_GENERATION == 'B'
        us_command('d', 0x30);
        command_xy1(x, y);
        command(0);
        command(len);
        while (len--)
            command(*buffer++);
    #endif
}

void Noritake_VFD_GU7000::GU7000_drawImage(unsigned x, uint8_t y, unsigned width, uint8_t height, const uint8_t *data) {
    #if NORITAKE_VFD_GENERATION == 'B'
        us_command('d', 0x21);
        command_xy1(x, y);
        command_xy1(width, height);
        command(0x01);
        for (unsigned i = 0; i<(height/8)*width; i++)
            command(data[i]);
    #endif
}
void Noritake_VFD_GU7000::GU7000_drawImage(unsigned x, uint8_t y, ImageMemoryArea area, unsigned long address, uint8_t srcHeight, unsigned width, uint8_t height, unsigned offsetx, unsigned offsety) {
    #if NORITAKE_VFD_GENERATION == 'B'
        if (height > NORITAKE_VFD_HEIGHT) return;
        us_command('d', 0x20);
        command_xy1(x, y);
        command(area);
        command(address);
        command(address>>8);
        command(address>>16);
        command(srcHeight/8);
        command(srcHeight/8>>8);
        command_xy1(offsetx, offsety);
        command_xy1(width, height);
        command(0x01);
    #endif
}

void Noritake_VFD_GU7000::GU7000_drawImage(unsigned x, uint8_t y, ImageMemoryArea area, unsigned long address, unsigned width, uint8_t height) {
    #if NORITAKE_VFD_GENERATION == 'B'
        GU7000_drawImage(x, y, area, address, (height + 7) & ~7, width, height, 0, 0);
    #endif
}


void Noritake_VFD_GU7000::GU7000_drawImage_p(unsigned width, uint8_t height, const uint8_t *data) {
    if (height > NORITAKE_VFD_HEIGHT) return;
    us_command('f', 0x11);
    command_xy(width, height);
    command((uint8_t) 1);
    for (unsigned i = 0; i<(height/8)*width; i++)
        command(pgm_read_byte(data+i));
}

void Noritake_VFD_GU7000::GU7000_drawImage_p(unsigned x, uint8_t y, unsigned width, uint8_t height, const uint8_t *data) {
    #if NORITAKE_VFD_GENERATION == 'B'
        us_command('d', 0x21);
        command_xy1(x, y);
        command_xy1(width, height);
        command(0x01);
        for (unsigned i = 0; i<(height/8)*width; i++)
            command(pgm_read_byte(data+i));
    #endif
}
*/

