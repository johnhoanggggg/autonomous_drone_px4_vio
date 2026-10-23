"""Join three compact iso panels (render_octomap_iso_seq.py --compact --report) into one row with a
shared legend underneath, for a report figure whose caption carries the explanation.

Usage: python make_iso_row_report.py <panel_stem> <out_stem>
Reads <panel_stem>_1..3.png; writes <out_stem>.png and <out_stem>.pdf.
"""
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from PIL import Image  # noqa: E402

panel_stem, out_stem = sys.argv[1:3]
SURF = (252, 252, 251)
raw = [Image.open(f'{panel_stem}_{k}.png').convert('RGB') for k in (1, 2, 3)]


def content_box(im, pad=20):
    a = np.asarray(im).astype(int)
    ys, xs = np.nonzero(np.abs(a - np.array(SURF)).sum(axis=2) > 30)
    return (max(xs.min() - pad, 0), max(ys.min() - pad, 0), min(xs.max() + pad, im.size[0]), min(ys.max() + pad, im.size[1]))


# one crop for all three so the scene keeps the same size and place in every panel
b = [content_box(im) for im in raw]
union = (min(x[0] for x in b), min(x[1] for x in b), max(x[2] for x in b), max(x[3] for x in b))
panels = [np.asarray(im.crop(union)) for im in raw]
h, w = panels[0].shape[:2]
fig = plt.figure(figsize=(3 * w / h * 4.2, 4.2 + 0.7), facecolor='#fcfcfb')
for k, pnl in enumerate(panels):
    ax = fig.add_axes([k / 3, 0.7 / 4.9, 1 / 3, 4.2 / 4.9])
    ax.imshow(pnl)
    ax.set_axis_off()
handles = [Line2D([], [], color='#eb6834', lw=3), Line2D([], [], color='#0b0b0b', lw=2),
           Line2D([], [], marker='s', ls='', ms=11, color='#2a78d6')]
fig.legend(handles, ['flown track', 'drone + 0.5 m cage', 'obstacle (darker = higher)'], loc='lower center',
           ncol=3, fontsize=13, frameon=False, bbox_to_anchor=(0.5, 0.0), columnspacing=2.0)
for ext in ('png', 'pdf'):
    fig.savefig(f'{out_stem}.{ext}', dpi=300, facecolor='#fcfcfb')
print('wrote', out_stem + '.png/.pdf')
