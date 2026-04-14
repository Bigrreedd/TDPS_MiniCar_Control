# -*- coding: utf-8 -*-
"""Generate SSD1306-style 16x16 column-major bitmaps (32 bytes per char)."""
import os
from PIL import Image, ImageDraw, ImageFont

chars = "\u4f4d\u89d2\u538b\u5149\u901f\u529f\u7535\u91cf"  # 位角压光速功电量
# 位 角 压 光 速 功 电 量

def to_ssd1306_bytes(img):
    w, h = img.size
    assert w == 16 and h == 16
    px = img.load()
    out = []
    for x in range(16):
        b0 = 0
        b1 = 0
        for y in range(8):
            v = 1 if px[x, y] else 0
            b0 |= v << y
        for y in range(8, 16):
            v = 1 if px[x, y] else 0
            b1 |= v << (y - 8)
        out.append(b0)
        out.append(b1)
    return out

def main():
    font_paths = [
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\simsun.ttc",
    ]
    fp = None
    for p in font_paths:
        if os.path.isfile(p):
            fp = p
            break
    if not fp:
        raise SystemExit("No Chinese font found")
    font = ImageFont.truetype(fp, 13)
    all_bytes = []
    for ch in chars:
        im = Image.new("L", (16, 16), 0)
        dr = ImageDraw.Draw(im)
        bbox = dr.textbbox((0, 0), ch, font=font)
        w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
        ox = max(0, (16 - w) // 2 - bbox[0])
        oy = max(0, (16 - h) // 2 - bbox[1])
        dr.text((ox, oy), ch, font=font, fill=255)
        im = im.point(lambda p: 255 if p > 128 else 0, mode="1")
        arr = to_ssd1306_bytes(im)
        all_bytes.append(arr)
    print("/* auto-generated - 位角压光速功电量 order */")
    names = ["wei", "jiao", "ya", "guang", "su", "gong", "dian", "liang"]
    for name, arr in zip(names, all_bytes):
        print(f"static const uint8_t CN_{name}[32] = {{")
        print(", ".join(f"0x{b:02X}" for b in arr))
        print("};")
    print("\n/* combined table CN_ORDER: 0位 1角 2压 3光 4速 5功 6电 7量 */")
    print("static const uint8_t * const CN_GLYPHS[8] = {")
    for name in names:
        print(f"    CN_{name},")
    print("};")

if __name__ == "__main__":
    main()
