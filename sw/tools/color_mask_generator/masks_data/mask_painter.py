"""
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!! WARNING: This tool has been generated with the assistance of the LLM Chatbot CLaude. The tool has been made to help!!
!! simply with the laborious hand labeling of masks used for training. It does not use any logic that has been used   !!
!! to generate the NN algorithms and is simply a GUI to simplify this laborious and monotonous task                   !!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 GUI tool to paint masks for object detection training data.

Controls:
    Left mouse button = paint
    Right mouse button  = erase
    Scroll wheel  = zoom in / out
    Middle mouse button drag = pan
    Brush size slider  = change brush radius
    Overlay opacity slider   = change green overlay transparency
"""

import tkinter as tk
from tkinter import filedialog, messagebox
import cv2
import numpy as np
from PIL import Image, ImageTk
import os


class MaskPainter:
    def __init__(self, root):
        self.root = root
        self.root.title("Mask Painter — Orange Pole Labeller")
        self.root.configure(bg="#1e1e1e")

        # State
        self.img_path  = None
        self.img_orig  = None    # original BGR image (numpy)
        self.mask      = None    # H×W uint8, 0 or 255
        self.scale     = 1.0    # current zoom level
        self.offset_x  = 0      # pan offset in canvas pixels
        self.offset_y  = 0
        self.drawing   = False
        self.erasing   = False
        self.panning   = False
        self.last_cx   = None   # last canvas x (for line drawing)
        self.last_cy   = None
        self.pan_start_x = 0
        self.pan_start_y = 0

        self._build_ui()

    # UI
    def _build_ui(self):
        toolbar = tk.Frame(self.root, bg="#2d2d2d", pady=6)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        btn_style = dict(bg="#444", fg="white", relief=tk.FLAT,
                         padx=12, pady=4, cursor="hand2",
                         activebackground="#666", activeforeground="white",
                         font=("Helvetica", 11))

        tk.Button(toolbar, text="📂  Open Image",
                  command=self.open_image, **btn_style).pack(side=tk.LEFT, padx=6)
        tk.Button(toolbar, text="💾  Save Mask",
                  command=self.save_mask, **btn_style).pack(side=tk.LEFT, padx=6)
        tk.Button(toolbar, text="🗑  Clear Mask",
                  command=self.clear_mask, **btn_style).pack(side=tk.LEFT, padx=6)
        tk.Button(toolbar, text="🔍  Reset Zoom",
                  command=self.reset_zoom, **btn_style).pack(side=tk.LEFT, padx=6)

        # Brush size
        tk.Label(toolbar, text="Brush:", bg="#2d2d2d", fg="#ccc",
                 font=("Helvetica", 11)).pack(side=tk.LEFT, padx=(20, 4))
        self.brush_var = tk.IntVar(value=12)
        tk.Scale(toolbar, from_=2, to=60, orient=tk.HORIZONTAL,
                 variable=self.brush_var, bg="#2d2d2d", fg="white",
                 highlightthickness=0, troughcolor="#555",
                 length=140).pack(side=tk.LEFT)

        # Overlay opacity
        tk.Label(toolbar, text="Overlay:", bg="#2d2d2d", fg="#ccc",
                 font=("Helvetica", 11)).pack(side=tk.LEFT, padx=(20, 4))
        self.opacity_var = tk.DoubleVar(value=0.45)
        tk.Scale(toolbar, from_=0.0, to=1.0, resolution=0.05,
                 orient=tk.HORIZONTAL, variable=self.opacity_var,
                 bg="#2d2d2d", fg="white", highlightthickness=0,
                 troughcolor="#555", length=120,
                 command=lambda _: self._refresh_canvas()).pack(side=tk.LEFT)

        # Zoom label
        self.zoom_var = tk.StringVar(value="Zoom: 100%")
        tk.Label(toolbar, textvariable=self.zoom_var, bg="#2d2d2d", fg="#aaa",
                 font=("Helvetica", 10)).pack(side=tk.LEFT, padx=(20, 4))

        # Legend
        tk.Label(toolbar, text="LMB=paint  RMB=erase  Scroll=zoom  MMB=pan",
                 bg="#2d2d2d", fg="#777",
                 font=("Helvetica", 10)).pack(side=tk.RIGHT, padx=12)

        # Status bar
        self.status_var = tk.StringVar(value="Open an image to start.")
        tk.Label(self.root, textvariable=self.status_var,
                 bg="#1e1e1e", fg="#aaa", anchor="w",
                 font=("Helvetica", 10)).pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=4)

        # Scrollable canvas frame
        canvas_frame = tk.Frame(self.root, bg="#1e1e1e")
        canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(canvas_frame, bg="#111",
                                cursor="crosshair", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Mouse bindings
        self.canvas.bind("<ButtonPress-1>",   self._on_lmb_press)
        self.canvas.bind("<B1-Motion>",        self._on_lmb_drag)
        self.canvas.bind("<ButtonRelease-1>",  self._on_release)
        self.canvas.bind("<ButtonPress-3>",    self._on_rmb_press)
        self.canvas.bind("<B3-Motion>",        self._on_rmb_drag)
        self.canvas.bind("<ButtonRelease-3>",  self._on_release)

        # Zoom — scroll wheel (Linux: Button-4/5, Windows/Mac: MouseWheel)
        self.canvas.bind("<MouseWheel>",       self._on_mousewheel)
        self.canvas.bind("<Button-4>",         self._on_scroll_up)
        self.canvas.bind("<Button-5>",         self._on_scroll_down)

        # Pan — middle mouse button
        self.canvas.bind("<ButtonPress-2>",    self._on_pan_start)
        self.canvas.bind("<B2-Motion>",        self._on_pan_drag)
        self.canvas.bind("<ButtonRelease-2>",  self._on_pan_end)

        self.canvas.bind("<Configure>",        self._on_resize)

    # File operations
    def open_image(self):
        path = filedialog.askopenfilename(
            title="Select an image",
            filetypes=[
                ("All image types", "*.jpg *.jpeg *.png *.bmp *.tiff *.tif *.webp"),
                ("JPEG",            "*.jpg *.jpeg"),
                ("PNG",             "*.png"),
                ("BMP",             "*.bmp"),
                ("TIFF",            "*.tiff *.tif"),
                ("WebP",            "*.webp"),
                ("All files",       "*.*"),
            ]
        )
        if not path:
            return

        self.img_path = path
        self.img_orig = cv2.imread(path)
        if self.img_orig is None:
            messagebox.showerror("Error", f"Could not read:\n{path}")
            return

        h, w = self.img_orig.shape[:2]
        self.mask = np.zeros((h, w), dtype=np.uint8)

        # Check for existing mask and offer to reload it
        mask_path = os.path.splitext(path)[0] + "_mask.jpg"
        if os.path.exists(mask_path):
            if messagebox.askyesno("Existing mask found",
                                   f"Load existing mask?\n{os.path.basename(mask_path)}"):
                existing = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
                if existing is not None and existing.shape == (h, w):
                    self.mask = existing

        self.status_var.set(f"Loaded: {os.path.basename(path)}  ({w}×{h})")
        self._fit_image()
        self._refresh_canvas()

    def save_mask(self):
        if self.mask is None:
            messagebox.showwarning("No mask", "Paint a mask first.")
            return

        folder = filedialog.askdirectory(title="Choose folder to save mask into")
        if not folder:
            return

        original_name = os.path.splitext(os.path.basename(self.img_path))[0]
        mask_filename = original_name + "_mask.jpg"
        mask_path = os.path.join(folder, mask_filename)

        mask_3ch = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(mask_path, mask_3ch)
        self.status_var.set(f"✅  Saved → {mask_filename}")
        messagebox.showinfo("Saved", f"Mask saved to:\n{mask_path}")

    def clear_mask(self):
        if self.mask is not None:
            self.mask[:] = 0
            self._refresh_canvas()

    def reset_zoom(self):
        self._fit_image()
        self._refresh_canvas()

    # Coordinate helpers
    def _canvas_to_image(self, cx, cy):
        """Convert canvas pixel → original image pixel accounting for pan/zoom."""
        ix = int((cx - self.offset_x) / self.scale)
        iy = int((cy - self.offset_y) / self.scale)
        return ix, iy

    # Drawing
    def _paint(self, cx, cy, value):
        if self.mask is None:
            return
        ix, iy = self._canvas_to_image(cx, cy)
        # Brush radius in image pixels (constant regardless of zoom)
        r = max(1, self.brush_var.get())
        cv2.circle(self.mask, (ix, iy), r, int(value), -1)

        if self.last_cx is not None:
            lx, ly = self._canvas_to_image(self.last_cx, self.last_cy)
            cv2.line(self.mask, (lx, ly), (ix, iy), int(value), thickness=max(1, r * 2))

        self.last_cx, self.last_cy = cx, cy
        self._refresh_canvas()

    def _on_lmb_press(self, e):
        self.drawing = True
        self.last_cx = self.last_cy = None
        self._paint(e.x, e.y, 255)

    def _on_lmb_drag(self, e):
        if self.drawing:
            self._paint(e.x, e.y, 255)

    def _on_rmb_press(self, e):
        self.erasing = True
        self.last_cx = self.last_cy = None
        self._paint(e.x, e.y, 0)

    def _on_rmb_drag(self, e):
        if self.erasing:
            self._paint(e.x, e.y, 0)

    def _on_release(self, _):
        self.drawing = self.erasing = False
        self.last_cx = self.last_cy = None

    # Zoom
    ZOOM_STEP = 1.15
    ZOOM_MIN  = 0.05
    ZOOM_MAX  = 20.0

    def _zoom(self, factor, cx, cy):
        """Zoom by factor, keeping canvas point (cx, cy) fixed."""
        if self.img_orig is None:
            return
        new_scale = max(self.ZOOM_MIN, min(self.ZOOM_MAX, self.scale * factor))
        if new_scale == self.scale:
            return
        # Adjust offset so the point under the cursor stays fixed
        self.offset_x = cx - (cx - self.offset_x) * (new_scale / self.scale)
        self.offset_y = cy - (cy - self.offset_y) * (new_scale / self.scale)
        self.scale = new_scale
        self.zoom_var.set(f"Zoom: {int(self.scale * 100)}%")
        self._refresh_canvas()

    def _on_mousewheel(self, e):
        factor = self.ZOOM_STEP if e.delta > 0 else 1 / self.ZOOM_STEP
        self._zoom(factor, e.x, e.y)

    def _on_scroll_up(self, e):
        self._zoom(self.ZOOM_STEP, e.x, e.y)

    def _on_scroll_down(self, e):
        self._zoom(1 / self.ZOOM_STEP, e.x, e.y)

    # Pan
    def _on_pan_start(self, e):
        self.panning = True
        self.pan_start_x = e.x - self.offset_x
        self.pan_start_y = e.y - self.offset_y
        self.canvas.config(cursor="fleur")

    def _on_pan_drag(self, e):
        if self.panning:
            self.offset_x = e.x - self.pan_start_x
            self.offset_y = e.y - self.pan_start_y
            self._refresh_canvas()

    def _on_pan_end(self, _):
        self.panning = False
        self.canvas.config(cursor="crosshair")

    # Rendering
    def _fit_image(self):
        """Scale + centre image to fit the canvas, reset pan."""
        if self.img_orig is None:
            return
        cw = self.canvas.winfo_width()  or 900
        ch = self.canvas.winfo_height() or 600
        ih, iw = self.img_orig.shape[:2]
        self.scale = min(cw / iw, ch / ih)
        # Centre the image
        self.offset_x = (cw - iw * self.scale) / 2
        self.offset_y = (ch - ih * self.scale) / 2
        self.zoom_var.set(f"Zoom: {int(self.scale * 100)}%")

    def _on_resize(self, _):
        if self.img_orig is not None:
            self._fit_image()
        self._refresh_canvas()

    def _refresh_canvas(self):
        if self.img_orig is None:
            return

        ih, iw = self.img_orig.shape[:2]
        dw = max(1, int(iw * self.scale))
        dh = max(1, int(ih * self.scale))

        # Blend original + green mask overlay
        display = self.img_orig.copy()
        if self.mask is not None and self.mask.any():
            alpha   = self.opacity_var.get()
            overlay = display.copy()
            overlay[self.mask == 255] = [0, 220, 0]
            cv2.addWeighted(overlay, alpha, display, 1 - alpha, 0, display)

        # Resize to current zoom level
        interp  = cv2.INTER_NEAREST if self.scale > 2 else cv2.INTER_AREA
        display = cv2.resize(display, (dw, dh), interpolation=interp)
        rgb     = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        self._tk_img = ImageTk.PhotoImage(Image.fromarray(rgb))

        self.canvas.delete("all")
        self.canvas.create_image(int(self.offset_x), int(self.offset_y),
                                 anchor=tk.NW, image=self._tk_img)


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1100x700")
    app = MaskPainter(root)
    root.mainloop()