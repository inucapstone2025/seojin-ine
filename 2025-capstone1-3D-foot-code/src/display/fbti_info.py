import os
from PIL import Image

# ------------------------------
# QR 이미지 저장 폴더
# ------------------------------
QR_DIR = os.path.join(os.path.dirname(__file__), "../assets/fbti_qr")

# ------------------------------
# BTI 조합별 QR 이미지 매핑
# ------------------------------
fbti_qr_map = {
    "EWIH": os.path.join(QR_DIR, "EWIH.png"),
    "EWIL": os.path.join(QR_DIR, "EWIL.png"),
    "EWSH": os.path.join(QR_DIR, "EWSH.png"),
    "EWSL": os.path.join(QR_DIR, "EWSL.png"),
    "ENIH": os.path.join(QR_DIR, "ENIH.png"),
    "ENIL": os.path.join(QR_DIR, "ENIL.png"),
    "ENSH": os.path.join(QR_DIR, "ENSH.png"),
    "ENSL": os.path.join(QR_DIR, "ENSL.png"),
    "GWIH": os.path.join(QR_DIR, "GWIH.png"),
    "GWIL": os.path.join(QR_DIR, "GWIL.png"),
    "GWSH": os.path.join(QR_DIR, "GWSH.png"),
    "GWSL": os.path.join(QR_DIR, "GWSL.png"),
    "GNIH": os.path.join(QR_DIR, "GNIH.png"),
    "GNIL": os.path.join(QR_DIR, "GNIL.png"),
    "GNSH": os.path.join(QR_DIR, "GNSH.png"),
    "GNSL": os.path.join(QR_DIR, "GNSL.png"),
    "RWIH": os.path.join(QR_DIR, "RWIH.png"),
    "RWIL": os.path.join(QR_DIR, "RWIL.png"),
    "RWSH": os.path.join(QR_DIR, "RWSH.png"),
    "RWSL": os.path.join(QR_DIR, "RWSL.png"),
    "RNIH": os.path.join(QR_DIR, "RNIH.png"),
    "RNIL": os.path.join(QR_DIR, "RNIL.png"),
    "RNSH": os.path.join(QR_DIR, "RNSH.png"),
    "RNSL": os.path.join(QR_DIR, "RNSL.png"),
}

# ------------------------------
# QR 이미지 불러오기
# ------------------------------
def get_qr_image(bti_combination: str) -> Image.Image:
    """
    BTI 조합 이름으로 QR 이미지 열기
    """
    path = fbti_qr_map.get(bti_combination)
    if path and os.path.exists(path):
        return Image.open(path)
    else:
        raise FileNotFoundError(f"{bti_combination}에 대한 QR 이미지가 없습니다.")
