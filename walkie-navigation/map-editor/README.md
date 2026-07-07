# Walkie Map Editor

Static web tool to post-process Nav2 SLAM maps: erase/draw pixels, label
furniture, mark no-go zones, export a Nav2-ready keepout layer.

Live: https://map-editor-two-eta.vercel.app

## Run locally

No build, no install. Static files only.

```bash
cd walkie-navigation/map-editor
python3 -m http.server 8080
```

Open <http://localhost:8080>. Chrome / Edge / Brave recommended for
- `webkitdirectory` folder picker
- `showDirectoryPicker()` write-to-disk export

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

**Pixel** — operate on the raster image, brush slider on the toolbar.

| Tool | Effect |
|---|---|
| Pen | paint occupied (black, value 0) |
| Eraser | paint free (white, value 254) |
| Restore | revert pixels to the value in `_og.pgm` |

- Brush `1` paints exactly one pixel. `2+` is a filled disk of that diameter.
- Fast mouse moves are interpolated, so a straight drag draws a continuous
  line (no gaps).

**Shape** — store coordinates in **world meters**, resolution-independent.

| Tool | Click behaviour |
|---|---|
| Select | click an element on the map to highlight it |
| Point | single click places one point |
| Rect | press and drag two corners |
| Polygon | left-click adds vertex; click near start to close |
| No-Go | polygon tagged as `nogo`, exported into `*_keepout.pgm` |

Closed polygons or rectangles can ALSO be flagged as no-go without changing
their label: select the element in the sidebar and click the **nogo** button.
The shape keeps its original label, and its area is rasterized into
`*_keepout.pgm` alongside dedicated no-go polygons.

A dashed rubber-band line follows the cursor while drawing.

**Viewport**

- Mouse wheel — zoom at cursor
- Middle-drag, Alt-drag, or Ctrl-drag — pan
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

**Visibility filters**

Two filter panels in the sidebar — **Label** and **Drawing kind**. Untick to
hide matching elements from the canvas and dim them in the list. Hidden
elements are not click-selectable but stay in the export.

The **IDs** checkbox in the Elements summary toggles `#id` tags on the canvas.

Closing the tab with unsaved edits triggers a browser confirm. Export first
to silence it.

## Export

Click **Export**. The browser asks where to save. The editor creates a
subfolder named `<prefix>_YYYYMMDD_HHMM/` inside the chosen directory and
writes five files into it:

| File | Content |
|---|---|
| `<prefix>_og.pgm` | original PGM (never mutated, kept for Restore) |
| `<prefix>.pgm` | edited PGM |
| `<prefix>.yaml` | Nav2 metadata, `image:` points at `<prefix>.pgm` |
| `<prefix>_element.json` | labels + elements (world coords) |
| `<prefix>_keepout.pgm` | white image with all no-go and `asNogo` areas filled black |

Re-import the exported subfolder to round-trip.

**Browsers without `showDirectoryPicker`** (Firefox, Safari) fall back to
five individual downloads, each prefixed with the folder name so you can
group them manually.

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
      "asNogo": false,
      "coords": [[x_m, y_m], ...]
    }
  ]
}
```

`coords` are in the map frame (Nav2 world coords, Y-up, meters).

## Known limitations

- **Safari** has weak `webkitdirectory` support. Fallback: shift/cmd-click to
  pick the files individually in the folder dialog.
- **Firefox / Safari** lack `showDirectoryPicker`, so export falls back to
  five separate downloads.
- **Restore tool** requires `*_og.pgm` to revert to a pristine baseline.
  Without it, Restore reverts only to the as-loaded state of the editable PGM.
- **Vertex editing** after a polygon is committed is not supported — delete
  and redraw.
- Not profiled above ~5000 px on a side.

## Self-check

Open dev console and run `_test()` — round-trips a tiny PGM, parses a YAML
sample, rasterizes a square. Logs `self-check ok` on success.
