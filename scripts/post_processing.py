from pykml import parser
import matplotlib.pyplot as plt
import rasterio
from rasterio.transform import from_origin
from PIL import Image
import numpy as np
import os
import sys
import zipfile
import pyproj

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

#walls Lat, Lon of corners
coords_latlon = [
    (44.09601434757111, 9.864511297776231),
    (44.09614116689923, 9.865068063673895),
    (44.09571342274439, 9.865252329430948),
    (44.09558901666539, 9.864688844060744)
]



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

# Convert all lat/lon to UTM coordinates
proj = pyproj.Proj(proj='utm', zone=32, ellps='WGS84')  # Use correct UTM zone for your region

def latlon_to_utm(lat, lon):
    return proj(lon, lat)

# Convert KML points to UTM
utm_point_coords = [(latlon_to_utm(lat, lon)[0], latlon_to_utm(lat, lon)[1], pm) for lat, lon, pm in point_coords]
utm_line_coords_list = [([(latlon_to_utm(lat, lon)[0], latlon_to_utm(lat, lon)[1]) for lat, lon in line], pm)
                        for line, pm in line_coords_list]

# Convert wall corners to UTM
wall_coords_utm = [latlon_to_utm(lat, lon) for lat, lon in coords_latlon]

# Determine UTM bounds
all_x = [x for x, _, _ in utm_point_coords] + [x for line, _ in utm_line_coords_list for x, _ in line] + [x for x, y in wall_coords_utm]
all_y = [y for _, y, _ in utm_point_coords] + [y for line, _ in utm_line_coords_list for _, y in line] + [y for x, y in wall_coords_utm]
min_x, max_x = min(all_x), max(all_x)
min_y, max_y = min(all_y), max(all_y)

# Plot with meter-scaled axis
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(min_x - 5, max_x + 5)
ax.set_ylim(min_y - 5, max_y + 5)

# Plot KML points
for x, y, pm in utm_point_coords:
    ax.plot(x, y, 'ro', markersize=4)
    name = pm.find('kml:name', namespaces=NS)
    if name is not None and name.text:
        ax.text(x, y + 1, name.text, fontsize=8, ha='center', va='bottom')

# Plot KML lines
for coords, pm in utm_line_coords_list:
    xs, ys = zip(*coords)
    ax.plot(xs, ys, 'b-', linewidth=1)

# Plot wall polyline (open shape)
wall_xs, wall_ys = zip(*wall_coords_utm)
ax.plot(wall_xs, wall_ys, 'k-', linewidth=2, label="Walls")

# Add axis and grid
ax.set_xlabel("Easting (m)")
ax.set_ylabel("Northing (m)")
ax.set_title("2D Map")
ax.grid(True)
ax.legend()

# Add cardinal direction labels on edges
ax.text((min_x + max_x) / 2, max_y + 2, 'NORTH', ha='center', va='bottom', fontsize=12, fontweight='bold')
ax.text((min_x + max_x) / 2, min_y - 2, 'SOUTH', ha='center', va='top', fontsize=12, fontweight='bold')
ax.text(min_x - 2, (min_y + max_y) / 2, 'WEST', ha='right', va='center', fontsize=12, fontweight='bold', rotation=90)
ax.text(max_x + 2, (min_y + max_y) / 2, 'EAST', ha='left', va='center', fontsize=12, fontweight='bold', rotation=270)


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
try:
    img = Image.open(OUTPUT_IMAGE).convert("RGB")
    img_np = np.array(img)
except Exception as e:
    print(f"❌ Error processing image: {e}")
    sys.exit(1)

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
