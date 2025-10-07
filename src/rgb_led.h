#pragma once

#include <LiteLED.h>

union rgb_s 
{
    struct __attribute__((packed)) {
        uint8_t blue, green, red;
    };
    uint32_t color;
} ;

union Hsv;

union Rgb {
    struct __attribute__((packed)) {
        uint8_t g, r, b, a;
    };
    uint32_t value;

    Rgb(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
        : g(g)
        , r(r)
        , b(b)
        , a(a) {}
    Rgb(const Hsv& c);
    Rgb(const Rgb&) = default;
    Rgb& operator=(const Rgb& rgb) {
        swap(rgb);
        return *this;
    }
    Rgb& operator=(const Hsv& hsv);
    Rgb operator+(const Rgb& in) const;
    Rgb& operator+=(const Rgb& in);
    Rgb operator-(const Rgb& in) const;
    Rgb& operator-=(const Rgb& in);
    bool operator==(const Rgb& in) const { return in.value == value; }
    Rgb& blend(const Rgb& in);
    void swap(const Rgb& o) { value = o.value; }
    void linearize() {
        r = channelGamma(r);
        g = channelGamma(g);
        b = channelGamma(b);
    }

    inline uint8_t IRAM_ATTR getGrb(int idx) {
        switch (idx) {
        case 0:
            return g;
        case 1:
            return r;
        case 2:
            return b;
        }
        __builtin_unreachable();
    }

    void stretchChannels(uint8_t maxR, uint8_t maxG, uint8_t maxB) {
        r = stretch(r, maxR);
        g = stretch(g, maxG);
        b = stretch(b, maxB);
    }

    void stretchChannelsEvenly(uint8_t max) { stretchChannels(max, max, max); }

private:
    uint8_t stretch(int value, uint8_t max) { return (value * max) >> 8; }

    uint8_t channelGamma(int channel) {
        /* The optimal gamma correction is x^2.8. However, this is expensive to
         * compute. Therefore, we use x^3 for gamma correction. Also, we add a
         * bias as the WS2812 LEDs do not turn on for values less than 4. */
        if (channel == 0)
            return channel;
        channel = channel * channel * channel * 251;
        channel >>= 24;
        return static_cast<uint8_t>(4 + channel);
    }
};

union Hsv {
    struct __attribute__((packed)) {
        uint8_t h, s, v, a;
    };
    uint32_t value;

    Hsv(uint8_t h, uint8_t s = 0, uint8_t v = 0, uint8_t a = 255)
        : h(h)
        , s(s)
        , v(v)
        , a(a) {}
    Hsv(const Rgb& r);
    Hsv& operator=(const Hsv& h) {
        swap(h);
        return *this;
    }
    Hsv& operator=(const Rgb& rgb);
    bool operator==(const Hsv& in) const { return in.value == value; }
    void swap(const Hsv& o) { value = o.value; }
};

int iRgbSqrt(int num) {
    // https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Binary_numeral_system_.28base_2.29
    assert("sqrt input should be non-negative" && num >= 0);
    assert("sqrt input should no exceed 16 bits" && num <= 0xFFFF);
    int res = 0;
    int bit = 1 << 16;
    while (bit > num)
        bit >>= 2;
    while (bit != 0) {
        if (num >= res + bit) {
            num -= res + bit;
            res = (res >> 1) + bit;
        } else
            res >>= 1;
        bit >>= 2;
    }
    return res;
}

Rgb::Rgb(const Hsv& y) {
    // https://stackoverflow.com/questions/24152553/hsv-to-rgb-and-back-without-floating-point-math-in-python
    // greyscale
    if (y.s == 0) {
        r = g = b = y.v;
        return;
    }

    const int region = y.h / 43;
    const int remainder = (y.h - (region * 43)) * 6;

    const int p = (y.v * (255 - y.s)) >> 8;
    const int q = (y.v * (255 - ((y.s * remainder) >> 8))) >> 8;
    const int t = (y.v * (255 - ((y.s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
    case 0:
        r = y.v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = y.v;
        b = p;
        break;
    case 2:
        r = p;
        g = y.v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = y.v;
        break;
    case 4:
        r = t;
        g = p;
        b = y.v;
        break;
    case 5:
        r = y.v;
        g = p;
        b = q;
        break;
    default:
        __builtin_trap();
    }

    a = y.a;
}

Rgb& Rgb::operator=(const Hsv& hsv) {
    Rgb r { hsv };
    swap(r);
    return *this;
}

Rgb Rgb::operator+(const Rgb& in) const {
    auto copy = *this;
    copy += in;
    return copy;
}

Rgb& Rgb::operator+=(const Rgb& in) {
    unsigned int red = r + in.r;
    r = (red < 255) ? red : 255;
    unsigned int green = g + in.g;
    g = (green < 255) ? green : 255;
    unsigned int blue = b + in.b;
    b = (blue < 255) ? blue : 255;
    return *this;
}

Rgb Rgb::operator-(const Rgb& in) const {
    auto copy = *this;
    copy -= in;
    return copy;
}

Rgb& Rgb::operator-=(const Rgb& in) {
    r = (in.r > r) ? 0 : r - in.r;
    g = (in.g > g) ? 0 : g - in.g;
    b = (in.b > b) ? 0 : b - in.b;
    return *this;
}

Rgb& Rgb::blend(const Rgb& in) {
    unsigned int inAlpha = in.a * (255 - a);
    unsigned int alpha = a + inAlpha;
    r = iRgbSqrt(((r * r * a) + (in.r * in.r * inAlpha)) / alpha);
    g = iRgbSqrt(((g * g * a) + (in.g * in.g * inAlpha)) / alpha);
    b = iRgbSqrt(((b * b * a) + (in.b * in.b * inAlpha)) / alpha);
    a = alpha;
    return *this;
}

static int up(int x) { return x * 255; }

Hsv::Hsv(const Rgb& r) {
    int min = std::min(r.r, std::min(r.g, r.b));
    int max = std::max(r.r, std::max(r.g, r.b));
    int chroma = max - min;

    v = max;
    if (chroma == 0) {
        h = s = 0;
        return;
    }

    s = up(chroma) / max;
    int hh;
    if (max == r.r)
        hh = (up(int(r.g) - int(r.b))) / chroma / 6;
    else if (max == r.g)
        hh = 255 / 3 + (up(int(r.b) - int(r.r))) / chroma / 6;
    else
        hh = 2 * 255 / 3 + (up(int(r.r) - int(r.g))) / chroma / 6;

    if (hh < 0)
        hh += 255;
    h = hh;

    a = r.a;
}

Hsv& Hsv::operator=(const Rgb& rgb) {
    Hsv h { rgb };
    swap(h);
    return *this;
}