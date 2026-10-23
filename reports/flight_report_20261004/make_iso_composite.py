"""Trim three compact iso panels and lay them out as a 2x2 square (with a key tile) and a 1x3 row.

Usage: python make_iso_composite.py <panel_stem> <out_stem> <title> <subtitle> <caption>
Reads <panel_stem>_1..3.png (from render_octomap_iso_seq.py --compact), rewrites them trimmed
and padded to squares, and writes <out_stem>_square.png and <out_stem>_row.png.
"""
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402

panel_stem, out_stem, title, subtitle, caption = sys.argv[1:6]
SURF = (252, 252, 251)
P = 1100


def content_box(im, pad=24):
    a = np.asarray(im.convert('RGB')).astype(int)
    ys, xs = np.nonzero(np.abs(a - np.array(SURF)).sum(axis=2) > 30)
    return (max(xs.min() - pad, 0), max(ys.min() - pad, 0),
            min(xs.max() + pad, im.size[0]), min(ys.max() + pad, im.size[1]))


raw = [Image.open(f'{panel_stem}_{k}.png').convert('RGB') for k in (1, 2, 3)]
# One crop rectangle for all three (the union), so the scene keeps the same size and place.
boxes = [content_box(im) for im in raw]
union = (min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes))
panels = [im.crop(union) for im in raw]
side = max(max(p.size) for p in panels)
tiles = []
for k, p in enumerate(panels, 1):
    sq = Image.new('RGB', (side, side), SURF)
    sq.paste(p, ((side - p.size[0]) // 2, (side - p.size[1]) // 2))
    sq.save(f'{panel_stem}_{k}.png')
    tiles.append(sq.resize((P, P), Image.LANCZOS))

fig = plt.figure(figsize=(5.5, 5.5), facecolor='#fcfcfb')
ax = fig.add_axes([0, 0, 1, 1])
ax.axis('off'); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
ax.text(0.08, 0.88, title, fontsize=18, fontweight='bold', color='#0b0b0b')
ax.text(0.08, 0.82, subtitle, fontsize=11.5, color='#52514e')
y = 0.70
for kind, col, lab in [('s', '#2a78d6', 'obstacle the drone flies through'),
                       ('line', '#eb6834', 'flown track so far'),
                       ('o', '#eb6834', 'takeoff'),
                       ('ring', '#0b0b0b', 'drone + 0.5 m cage')]:
    if kind == 's':
        ax.plot(0.12, y, 's', ms=15, color=col)
    elif kind == 'line':
        ax.plot([0.08, 0.16], [y, y], color=col, lw=4)
    elif kind == 'o':
        ax.plot(0.12, y, 'o', ms=12, mfc='#fcfcfb', mec=col, mew=2.5)
    else:
        ax.add_patch(plt.Circle((0.12, y), 0.035, fill=False, lw=2, color=col))
    ax.text(0.21, y, lab, va='center', fontsize=12.5, color='#0b0b0b')
    y -= 0.09
ax.text(0.08, y - 0.02, caption, fontsize=10.5, color='#52514e', va='top', linespacing=1.45)
key_path = f'{out_stem}_key_tmp.png'
fig.savefig(key_path, dpi=200, facecolor='#fcfcfb')
plt.close(fig)
key = Image.open(key_path).convert('RGB').resize((P, P), Image.LANCZOS)
import os  # noqa: E402
os.remove(key_path)

square = Image.new('RGB', (2 * P, 2 * P), SURF)
for i, img in enumerate(tiles + [key]):
    square.paste(img, ((i % 2) * P, (i // 2) * P))
square.save(f'{out_stem}_square.png')
row = Image.new('RGB', (3 * P, P), SURF)
for i, img in enumerate(tiles):
    row.paste(img, (i * P, 0))
row.save(f'{out_stem}_row.png')
print('trimmed', [p.size for p in panels], '->', f'{out_stem}_square.png', f'{out_stem}_row.png')
