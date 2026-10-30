#!/usr/bin/env python3
"""Build deterministic thumbnail sheets for manual review, never auto-label."""
import argparse
from pathlib import Path
from PIL import Image, ImageDraw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--recordings', type=Path, required=True)
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--stride', type=int, default=12)
    args = ap.parse_args()
    paths = []
    for run in args.runs:
        frames = sorted((args.recordings / run / 'front').glob('frame_*.png'))
        paths += frames[::args.stride]
    width, height, cols = 260, 220, 5
    rows = (len(paths) + cols - 1) // cols
    sheet = Image.new('RGB', (width * cols, height * rows), '#f3f3f3')
    draw = ImageDraw.Draw(sheet)
    for i, path in enumerate(paths):
        with Image.open(path) as src:
            thumb = src.convert('RGB')
            thumb.thumbnail((width - 8, height - 34))
            x, y = (i % cols) * width, (i // cols) * height
            sheet.paste(thumb, (x + 4, y + 4))
            label = f'{path.parents[1].name}/{path.name}'
            draw.text((x + 4, y + height - 26), label, fill='black')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.output)
    print(args.output, len(paths), sheet.size)


if __name__ == '__main__':
    main()
