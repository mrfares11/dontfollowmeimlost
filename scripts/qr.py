import os
import qrcode

labels = [
    "DOCK",
    "PHARMACY",
    "FIRE_STATION",
    "SUPERMARKET",
    "RESTAURANT",
    "HOUSE_1",
    "HOUSE_2",
    "HOUSE_3",
    "HOUSE_4",
    "HOUSE_5",
]

out_dir = "qr_codes"
os.makedirs(out_dir, exist_ok=True)

for label in labels:
    img = qrcode.make(label)
    img.save(os.path.join(out_dir, f"{label}.png"))

print("Saved QR codes in", out_dir)
