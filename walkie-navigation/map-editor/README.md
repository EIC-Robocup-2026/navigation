# Walkie Map Editor

Static web tool to post-process Nav2 SLAM maps: erase/draw pixels, label
furniture, mark no-go zones, export a Nav2-ready keepout layer.

## Run

No build, no install. Static files only.

```bash
cd walkie-navigation/map-editor
python3 -m http.server 8080
```

Open <http://localhost:8080>. Chrome / Edge / Brave recommended (folder
picker via `webkitdirectory`).

> Opening `index.html` directly with `file://` works in most browsers too.

## Load a map

Click **Load folder** and pick a folder containing:

| File | Required | Notes |
|---|---|---|
| `*.pgm` | yes | P5 (binary) Nav2 map |
| `*.yaml` | yes | Nav2 map metadata |
| `*_og.pgm` | no | original-pristine PGM, used by Restore tool |
| `*_element.json` | no | previously saved elements + custom labels |

`*_keepout.pgm` is ignored on load (regenerated on export).

## Tools

**Pixel** — operate on the raster image, brush radius slider on right.

| Tool | Effect |
|---|---|
| Pen | paint occupied (black, value 0) |
| Eraser | paint free (white, value 254) |
| Restore | revert pixels to the value in `_og.pgm` |

**Shape** — store coordinates in **world meters**, resolution-independent.

| Tool | Click behaviour |
|---|---|
| Select | click an element on the map to highlight it |
| Point | single click places one point |
| Rect | press and drag two corners |
| Polygon | left-click adds vertex; click near start to close |
| Line | left-click adds vertex; right-click or `Esc` to finish (open) |
| No-Go | polygon tagged as `nogo`, exported into `*_keepout.pgm` |

While drawing a polygon / line / no-go a dashed rubber-band line follows the
cursor.

Pick a label from the dropdown before drawing. Press **+** to add a custom
label (persists in `localStorage`).

**Viewport**

- Mouse wheel — zoom at cursor
- Middle-drag or Alt+drag — pan
- **Fit** button — fit the whole map
- Red/green crosshair = world origin `(0, 0)`
- Bottom-left = grid step size in meters / centimetres

**History**

- Ctrl/Cmd + Z — undo
- Ctrl/Cmd + Y or Ctrl/Cmd + Shift + Z — redo

**Selection**

- Switch to **Select** tool, then click an element on the map
- Or click an item in the sidebar
- Double-click a sidebar item to rename the label (Enter commits, Esc cancels)
- `Delete` / `Backspace` — remove the selected element
- `×` in the sidebar — remove that element

Closing the tab with unsaved edits triggers a browser confirm. Export first to
silence it.

## Export

Click **Export**. Five files download with the configured prefix:

| File | Content |
|---|---|
| `<prefix>_og.pgm` | original PGM (never mutated, kept for Restore) |
| `<prefix>.pgm` | edited PGM |
| `<prefix>.yaml` | Nav2 metadata, `image:` points at `<prefix>.pgm` |
| `<prefix>_element.json` | labels + elements (world coords) |
| `<prefix>_keepout.pgm` | white image with all `nogo` polygons filled black |

Re-import the exported folder to round-trip.

## `_element.json` format

```json
{
  "labels": ["table", "shelf", "..."],
  "elements": [
    {
      "id": "e1",
      "label": "shelf",
      "type": "rect | polygon | point | nogo",
      "closed": true,
      "coords": [[x_m, y_m], ...]
    }
  ]
}
```

`coords` are in the map frame (Nav2 world coords, Y-up, meters).

## Known limitations

- **Safari** has weak `webkitdirectory` support. Fallback: shift/cmd-click to
  pick the files individually in the folder dialog.
- **Restore tool** requires `*_og.pgm` to revert to a pristine baseline. Without
  it, Restore reverts only to the as-loaded state of the editable PGM.
- **Large maps** (~5000 px on any side) have not been profiled; pixel-edit
  redraw may stutter.
- **Vertex editing** after a polygon is committed is not supported. Delete and
  redraw.

## Self-check

Open dev console and run `_test()` — round-trips a tiny PGM, parses a YAML
sample, rasterizes a square. Logs `self-check ok` on success.
