안녕하세요. Python 전문가로서 답변드립니다.

**결론부터 말씀드리면, "DWG 파일을 Python만으로 직접 완벽하게 읽어서 PGM으로 변환"하는 것은 매우 어렵습니다.** DWG는 Autodesk의 비공개 바이너리 포맷이라 오픈소스 진영에서 완벽히 해석하기 힘들기 때문입니다.

따라서 품질 손실 없이(Vector to Raster) 스케일을 유지하며 변환하는 **가장 확실한 업계 표준 워크플로우(Workflow)**를 제안합니다.

**전체 프로세스:**

1. **전처리 (DWG  DXF):** ODA File Converter(무료)를 사용해 DWG를 개방형 포맷인 DXF로 변환합니다.
2. **렌더링 (DXF  Image):** Python의 `ezdxf`와 `Matplotlib`을 사용하여 벡터를 고해상도 이미지로 래스터화(Rasterize)합니다.
3. **변환 (Image  PGM):** `Pillow`를 사용해 PGM 포맷으로 저장합니다.

---

### 1단계: 사전 준비

필요한 라이브러리를 설치합니다.

```bash
pip install ezdxf matplotlib pillow

```

**중요:** DWG를 DXF로 변환하기 위해 [ODA File Converter](https://www.opendesign.com/guestfiles/oda_file_converter)를 설치해야 합니다. (CLI로 호출 가능)

---

### 2단계: Python 변환 코드 (Expert Level)

이 코드는 **스케일(Scale)과 해상도(DPI)**를 정밀하게 제어하여 품질 손실을 최소화하는 방식입니다. 로봇 맵(ROS) 등에서 사용할 때 중요한 `pixel_per_meter` (해상도) 개념을 적용했습니다.

```python
import sys
import subprocess
import os
from typing import Tuple

import ezdxf
from ezdxf.addons.drawing import RenderContext, Frontend
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
import matplotlib.pyplot as plt
from PIL import Image

def convert_dwg_to_dxf(dwg_path: str, output_dir: str) -> str:
    """
    ODA File Converter를 사용하여 DWG를 DXF로 변환합니다.
    ODA Converter가 설치되어 있어야 합니다.
    """
    # ODA File Converter의 기본 설치 경로 (OS에 맞게 수정 필요)
    # Ubuntu: '/usr/bin/ODAFileConverter'
    # Windows: 'C:\\Program Files\\ODA\\ODAFileConverter\\ODAFileConverter.exe'
    oda_path = '/usr/bin/ODAFileConverter' 
    
    if not os.path.exists(oda_path):
        raise FileNotFoundError(f"ODA Converter not found at {oda_path}")

    input_dir = os.path.dirname(dwg_path)
    filename = os.path.basename(dwg_path)
    dxf_filename = os.path.splitext(filename)[0] + ".dxf"
    dxf_path = os.path.join(output_dir, dxf_filename)

    # ODA CLI 실행: InputDir OutputDir OutputVer OutputType Recurse Audit
    # "ACAD2018" 버전, "DXF" 타입으로 변환
    cmd = [oda_path, input_dir, output_dir, "ACAD2018", "DXF", "0", "0", filename]
    
    print(f"Converting DWG to DXF...")
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    
    return dxf_path

def dxf_to_pgm(dxf_path: str, pgm_output_path: str, resolution: float = 0.05):
    """
    DXF를 읽어 PGM으로 변환합니다.
    
    :param dxf_path: DXF 파일 경로
    :param pgm_output_path: 저장할 PGM 파일 경로
    :param resolution: 픽셀 당 미터 (meters per pixel). 예: 0.05 (5cm) - 값이 작을수록 고화질
    """
    try:
        doc = ezdxf.readfile(dxf_path)
        msp = doc.modelspace()
    except IOError:
        print(f"Cannot read file: {dxf_path}")
        return
    except ezdxf.DXFStructureError:
        print(f"Invalid or corrupted DXF file: {dxf_path}")
        return

    # 1. 렌더링 컨텍스트 설정
    # 배경색을 흰색(White) 또는 검은색(Black)으로 설정
    ctx = RenderContext(doc)
    ctx.set_current_layout(msp)
    
    # 2. Matplotlib Backend 설정
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off() # 축 제거
    ax.set_aspect('equal') # 비율 유지
    
    out = MatplotlibBackend(ax)
    
    # 3. 렌더링 (Frontend)
    Frontend(ctx, out).draw_layout(msp, finalize=True)
    
    # 4. 바운딩 박스 계산 및 스케일 조정 (화질 보존의 핵심)
    # Matplotlib의 자동 스케일링 대신, 실제 좌표계를 픽셀로 매핑합니다.
    bbox = msp.get_bounding_box() # (min_x, min_y), (max_x, max_y)
    if not bbox:
        print("Empty drawing.")
        return

    min_x, min_y = bbox[0]
    max_x, max_y = bbox[1]
    
    width_m = max_x - min_x
    height_m = max_y - min_y
    
    # 필요한 이미지 크기 계산 (resolution에 따라)
    # DPI = 1인치당 픽셀 수. matplotlib은 기본적으로 inch 단위를 씀.
    # 하지만 여기서는 figure size를 직접 픽셀에 맞춰 조작합니다.
    
    img_width_px = int(width_m / resolution)
    img_height_px = int(height_m / resolution)
    
    # Matplotlib Figure 사이즈 조정 (inch 단위, 기본 dpi=100 가정 시)
    fig.set_size_inches(img_width_px / 100, img_height_px / 100)
    
    # 5. 이미지 버퍼로 저장 (PNG로 임시 렌더링)
    # dpi=100으로 설정하여 위에서 계산한 픽셀 수와 일치시킴
    fig.canvas.draw()
    
    # 6. Pillow를 사용하여 PGM 변환
    # Matplotlib 버퍼에서 이미지 가져오기
    buf = fig.canvas.buffer_rgba()
    image = Image.frombuffer('RGBA', (int(fig.bbox.width), int(fig.bbox.height)), buf, 'raw', 'RGBA', 0, 1)
    
    # Grayscale(L) 변환
    gray_image = image.convert("L")
    
    # PGM 저장 (Binary P5 포맷 권장)
    print(f"Saving to {pgm_output_path} (Size: {gray_image.size}, Resolution: {resolution} m/px)")
    gray_image.save(pgm_output_path)
    
    plt.close(fig)

# --- 실행 예시 ---
if __name__ == "__main__":
    # 1. 파일 경로 설정
    input_dwg = "./floor_plan.dwg"
    output_dir = "./output"
    os.makedirs(output_dir, exist_ok=True)
    
    # 2. DWG -> DXF 변환 (ODA Converter 필요, 없으면 DXF 파일 직접 지정)
    # dxf_file = convert_dwg_to_dxf(input_dwg, output_dir) 
    
    # 테스트를 위해 이미 DXF가 있다고 가정한다면:
    dxf_file = "floor_plan.dxf" # 실제 경로로 수정하세요
    
    # 3. DXF -> PGM 변환
    # resolution=0.01 (1cm 당 1픽셀) -> 매우 고화질
    dxf_to_pgm(dxf_file, "map_output.pgm", resolution=0.05)

```

### 핵심 포인트 (품질 및 스케일 유지)

1. **Resolution (m/px) 제어:**
* 단순히 이미지 크기(1024x768 등)를 지정하는 방식은 도면의 실제 비율(Scale)을 깨뜨립니다.
* 위 코드는 `resolution` 변수를 통해 **"실제 도면의 1m를 몇 픽셀로 표현할 것인가"**를 계산하여 `fig.set_size_inches`를 동적으로 조절합니다.
* `resolution=0.05`는 로봇 내비게이션(ROS Nav2)에서 표준으로 쓰이는 수치입니다. 더 높은 화질을 원하면 `0.01` 등으로 낮추십시오.


2. **ezdxf + Matplotlib:**
* `ezdxf`는 DXF 엔티티를 가장 정확하게 파싱하는 라이브러리입니다.
* `MatplotlibBackend`를 통해 선 두께, 색상, 곡선 등을 래스터화할 때 안티앨리어싱(Anti-aliasing) 처리가 되어 깨짐 현상이 적습니다.


3. **PGM 포맷:**
* PGM은 8-bit Grayscale 이미지입니다. `Pillow`의 `.convert("L")`을 사용하여 정확한 흑백/회색조 정보를 담습니다.



### 주의사항

* **ODA File Converter 경로:** 코드 내의 `oda_path`를 실제 설치된 경로로 변경해야 합니다.
* **좌표계:** 도면이 원점(0,0)에서 아주 멀리 떨어져 그려진 경우(예: 건축 도면이 GIS 좌표계 사용), 이미지 사이즈가 비정상적으로 커질 수 있습니다. 이 경우 `msp.get_bounding_box()` 전에 좌표를 원점으로 이동시키는 전처리가 필요할 수 있습니다.
