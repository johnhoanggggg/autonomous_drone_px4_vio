"""2x2 paper figure from the three frontier-simulation frames: (a) (b) top, (c) and a legend below.

Each panel's own legend is removed: in (a) and (c) it lies only over unexplored space, so it
is filled with the unexplored grey; in (b) it also covers a corner of the top-right room, which
is copied from (c) (same ground-truth map, already fully explored there), aligned through the
shared 0-140 cell axes. Titles are cropped (frame/coverage numbers go in the caption) and one
legend is redrawn with matplotlib in the fourth cell.
Usage: python make_frontier_grid.py <im-000.png> <im-001.png> <im-002.png> <out.png>
"""
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

GREY = np.array([89, 89, 89])


def legend_bbox(a):
    h, w, _ = a.shape
    x0 = int(0.55 * w)
    sub = a[:int(0.32 * h), x0:]
    g, sat = sub.mean(2), sub.max(2) - sub.min(2)
    m = (g > 200) & (g < 236) & (sat < 6)
    rows = np.where(m.sum(1) > 0.15 * m.shape[1])[0]
    r0, r1 = rows.min(), rows.max()
    cols = np.where(m[r0:r1 + 1].sum(0) > 0.3 * (r1 - r0))[0]
    return r0 - 4, r1 + 4, cols.min() + x0 - 4, cols.max() + x0 + 4


def plot_area(a):
    """Pixel box of the 0-140 x 0-140 map: extent of the unexplored-grey and map pixels."""
    near = np.abs(a - GREY).max(2) < 6
    rows = np.where(near.sum(1) > 0.3 * near.shape[1])[0]          # rows that cross the map
    cols = np.where(near.sum(0) > 0.3 * near.shape[0])[0]
    right = int(np.median([np.where(near[r])[0].max() for r in rows]))
    left = int(np.median([np.where(near[r])[0].min() for r in rows]))
    top = int(np.median([np.where(near[:, c])[0].min() for c in cols]))
    bottom = int(np.median([np.where(near[:, c])[0].max() for c in cols]))
    return top, bottom, left, right


imgs = [np.asarray(Image.open(p).convert('RGB')).astype(float) for p in sys.argv[1:4]]
boxes = [legend_bbox(a) for a in imgs]
areas = [plot_area(a) for a in imgs]


def to_data(i, r, c):
    y0, y1, x0, x1 = areas[i]
    return (c - x0) / (x1 - x0) * 140, (r - y0) / (y1 - y0) * 140


def to_px(i, x, y):
    y0, y1, x0, x1 = areas[i]
    return y0 + y / 140 * (y1 - y0), x0 + x / 140 * (x1 - x0)


for i, a in enumerate(imgs):
    r0, r1, c0, c1 = boxes[i]
    ay0, ay1, ax0, ax1 = areas[i]                  # keep the fill inside the axes frame
    r0, c1 = max(r0, ay0), min(c1, ax1)
    boxes[i] = (r0, r1, c0, c1)
    a[r0:r1 + 1, c0:c1 + 1] = GREY
    if i == 1:                                     # restore the room corner from panel (c)
        cr0, cr1, cc0, cc1 = boxes[2]
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                x, y = to_data(1, r, c)
                pr, pc = to_px(2, x, y)
                pr, pc = int(round(pr)), int(round(pc))
                if not (cr0 <= pr <= cr1 and cc0 <= pc <= cc1):
                    a[r, c] = imgs[2][pr, pc]

panels = []
for a in imgs:
    out = Image.fromarray(a.astype(np.uint8))
    g = out.convert('L')
    w, h = out.size
    top = 0
    for y in range(int(0.02 * h), int(0.2 * h)):
        if sum(g.getpixel((x, y)) < 100 for x in range(0, w, 2)) > 0.4 * (w // 2):
            top = y
            break
    c = out.crop((0, top - 1, w, h))
    ImageDraw.Draw(c).rectangle([0, 0, w, 0], fill=(255, 255, 255))
    panels.append(c)

H = 900
panels = [p.resize((int(p.width * H / p.height), H), Image.LANCZOS) for p in panels]
cw, lab, gap = max(p.width for p in panels), 70, 30

fig = plt.figure(figsize=(cw / 200, H / 200), dpi=200)
handles = [Line2D([], [], ls='', marker='o', ms=4, color='#1f77b4'),
           Line2D([], [], color='#1f77b4', lw=2.5),
           Line2D([], [], color='#ff7f0e', lw=2.5, ls='--'),
           Line2D([], [], ls='', marker='*', ms=16, color='#e377c2'),
           Line2D([], [], ls='', marker='x', ms=10, mew=2, color='#7f7f7f'),
           Line2D([], [], ls='', marker='o', ms=12, color='#ff7f0e')]
labels = ['frontier', 'drone trajectory', 'A* plan', 'selected NBV', 'candidate viewpoints', 'drone']
fig.legend(handles, labels, loc='center', fontsize=17, frameon=True, edgecolor='#9a9a96',
           fancybox=False, handlelength=2.2, labelspacing=0.8)
fig.canvas.draw()
leg = Image.frombuffer('RGBA', fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert('RGB')

font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 52)
s = Image.new('RGB', (2 * cw + gap, 2 * (H + lab) + gap), 'white')
for k, (p, (cx, cy)) in enumerate(zip(panels, [(0, 0), (cw + gap, 0), (0, H + lab + gap)])):
    s.paste(p, (cx + (cw - p.width) // 2, cy + lab))
    ImageDraw.Draw(s).text((cx + cw // 2, cy + 10), '(%s)' % 'abc'[k], fill='black', font=font, anchor='mt')
s.paste(leg, (cw + gap + (cw - leg.width) // 2, H + lab + gap + lab + (H - leg.height) // 2))
s.save(sys.argv[4])
print('wrote', sys.argv[4], s.size, 'plot areas', areas)
