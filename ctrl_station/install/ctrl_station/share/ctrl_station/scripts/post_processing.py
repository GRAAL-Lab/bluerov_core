from pykml import parser
import matplotlib.pyplot as plt
import rasterio
from rasterio.transform import from_origin
from PIL import Image
import numpy as np
import os
import sys
import zipfile

# --- Input arguments ---
if len(sys.argv) < 2:
    print("Usage: post_processing.py <mission_folder>")
    sys.exit(1)

MISSION_FOLDER = sys.argv[1]
KML_FILE = os.path.join(MISSION_FOLDER, "object_recognition_data.kml")
OUTPUT_DIR = os.path.join(MISSION_FOLDER, "map_information")
os.makedirs(OUTPUT_DIR, exist_ok=True)

OUTPUT_IMAGE = os.path.join(OUTPUT_DIR, "2d_map.png")
WORLD_FILE = os.path.join(OUTPUT_DIR, "map_output.pgw")
GEOTIFF_FILE = os.path.join(OUTPUT_DIR, "2d_map.tif")
KMZ_FILE = os.path.join(OUTPUT_DIR, "output_map.kmz")
WIDTH, HEIGHT = 800, 600
NS = {'kml': 'http://www.opengis.net/kml/2.2'}

def save_kmz(kml_path, output_path):
    with zipfile.ZipFile(output_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.write(kml_path, arcname=os.path.basename(kml_path))

# --- Parse KML ---
with open(KML_FILE, 'rt', encoding='utf-8') as f:
    root = parser.parse(f).getroot()

placemarks = root.findall('.//kml:Placemark', namespaces=NS)

point_coords = []
line_coords_list = []

# --- Extract Coordinates ---
for pm in placemarks:
    point = pm.find('.//kml:Point/kml:coordinates', namespaces=NS)
    if point is not None:
        lon, lat, *_ = point.text.strip().split(',')
        point_coords.append((float(lat), float(lon), pm))
        continue

    linestring = pm.find('.//kml:LineString/kml:coordinates', namespaces=NS)
    if linestring is not None:
        coords = []
        for coord in linestring.text.strip().split():
            lon, lat, *_ = coord.split(',')
            coords.append((float(lat), float(lon)))
        line_coords_list.append((coords, pm))

if not point_coords and not line_coords_list:
    print("No coordinates found in KML.")
    exit(1)

# --- Calculate Bounds ---
all_coords = [c[:2] for c in point_coords]
for line, _ in line_coords_list:
    all_coords.extend(line)

lats = [c[0] for c in all_coords]
lons = [c[1] for c in all_coords]
min_lat, max_lat = min(lats), max(lats)
min_lon, max_lon = min(lons), max(lons)

# --- Prevent ZeroDivisionError for single point KML ---
if max_lat - min_lat == 0:
    max_lat += 0.0001
    min_lat -= 0.0001

if max_lon - min_lon == 0:
    max_lon += 0.0001
    min_lon -= 0.0001

def latlon_to_pixel(lat, lon):
    x = int((lon - min_lon) / (max_lon - min_lon) * WIDTH)
    y = int((max_lat - lat) / (max_lat - min_lat) * HEIGHT)
    return x, y

# --- Plot ---
fig, ax = plt.subplots(figsize=(WIDTH / 100, HEIGHT / 100))
ax.set_xlim(-10, WIDTH + 10)
ax.set_ylim(-10, HEIGHT + 10)
ax.invert_yaxis()

# Plot points
for lat, lon, pm in point_coords:
    x, y = latlon_to_pixel(lat, lon)
    ax.plot(x, y, 'ro', markersize=4)
    name = pm.find('kml:name', namespaces=NS)
    if name is not None and name.text:
        ax.text(x, y + 5, name.text, fontsize=8, ha='center', va='top', color='black')        

# Plot lines
for coords, pm in line_coords_list:
    pixel_coords = [latlon_to_pixel(lat, lon) for lat, lon in coords]
    xs, ys = zip(*pixel_coords)
    ax.plot(xs, ys, 'b-', linewidth=1)

ax.set_axis_off()
plt.tight_layout()

# --- Save PNG ---
plt.savefig(OUTPUT_IMAGE, dpi=150)
plt.close()

# --- Save PGW (World File) ---
pixel_width = (max_lon - min_lon) / WIDTH
pixel_height = (max_lat - min_lat) / HEIGHT
with open(WORLD_FILE, "w") as f:
    f.write(f"{pixel_width:.12f}\n0.0\n0.0\n-{pixel_height:.12f}\n{min_lon:.12f}\n{max_lat:.12f}\n")

# --- Convert PNG to GeoTIFF using Rasterio ---
img = Image.open(OUTPUT_IMAGE).convert("RGB")
img_np = np.array(img)

transform = from_origin(min_lon, max_lat, pixel_width, pixel_height)

with rasterio.open(
    GEOTIFF_FILE,
    "w",
    driver="GTiff",
    height=img_np.shape[0],
    width=img_np.shape[1],
    count=3,
    dtype=img_np.dtype,
    crs="EPSG:4326",
    transform=transform,
) as dst:
    for i in range(3):  # Write R, G, B bands
        dst.write(img_np[:, :, i], i + 1)

# --- Save KMZ ---
save_kmz(KML_FILE, KMZ_FILE)

print(f"✅ All map files saved in: {OUTPUT_DIR}")
