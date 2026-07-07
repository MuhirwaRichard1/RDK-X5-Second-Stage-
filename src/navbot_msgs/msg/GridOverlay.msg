# Coarse per-cell overlay grid for the operator HUD — shared by pidnet's
# segmentation overlay and depthanything's depth overlay (same shape of
# data: a small category/scalar grid over one camera's image).
#
# Cell (r, c) covers image row-band r/rows..(r+1)/rows and column-band
# c/cols..(c+1)/cols; cells is row-major (index = r*cols + c).

uint8 KIND_PIDNET=0
uint8 KIND_DEPTH=1

std_msgs/Header header
string camera             # "front" | "left" | "right"
uint8 kind
uint8 rows
uint8 cols
uint8[] cells             # KIND_PIDNET: 0=unknown/1=free/2=blocked (Sectors-style)
                           # KIND_DEPTH: 0-255 normalized relative depth, brighter=closer
