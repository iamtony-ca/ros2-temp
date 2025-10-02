네, 물론 방법이 있습니다. 이미 인코딩 문제가 발생한 파일들을 Ubuntu에서 직접 수정하여 `colcon build`를 성공적으로 완료할 수 있습니다.

핵심은 **문제가 되는 파일들의 문자 인코딩을 Windows의 CP949(또는 EUC-KR)에서 Ubuntu의 표준인 UTF-8로 다시 변환**해주는 것입니다. 이 작업은 리눅스의 기본 명령어인 `iconv`를 사용하여 간단하게 해결할 수 있습니다.

아래 단계를 순서대로 따라 진행하시면 됩니다.

-----

### **해결 단계: 깨진 파일 인코딩을 UTF-8로 변환하기**

#### **1. 터미널 열고 ROS 2 작업 공간으로 이동**

먼저, 터미널을 열고 문제가 발생한 ROS 2의 작업 공간(workspace)으로 이동합니다.

```bash
cd ~/ros2_ws  # 사용자의 실제 작업 공간 경로에 맞게 수정하세요.
```

#### **2. (매우 중요) 원본 소스 코드 백업**

파일을 직접 수정하기 전에, 만일의 사태에 대비하여 `src` 폴더 전체를 백업하는 것이 안전합니다.

```bash
cp -r src src_backup
```

이제 `src_backup`이라는 폴더에 원본이 그대로 복사되었으므로, 안심하고 다음 단계를 진행할 수 있습니다.

#### **3. `iconv`를 사용하여 인코딩 변환 실행**

`UnicodeDecodeError`는 주로 `.msg`, `.srv`, `.action` 파일에서 발생합니다. 아래 명령어는 `src` 폴더 내의 모든 관련 파일을 찾아 자동으로 인코딩을 CP949에서 UTF-8로 변환해 줍니다.

아래 명령어를 복사하여 터미널에 붙여넣고 실행하세요.

```bash
find ./src -type f \( -name "*.msg" -o -name "*.srv" -o -name "*.action" \) -print0 | while IFS= read -r -d $'\0' file; do
    # 임시 파일로 먼저 변환을 시도합니다.
    iconv -f CP949 -t UTF-8 "$file" -o "${file}.tmp"
    
    # 변환이 성공적으로 완료되면, 원본 파일을 덮어씁니다.
    if [ $? -eq 0 ]; then
        mv "${file}.tmp" "$file"
        echo "인코딩 변환 완료: $file"
    else
        # 변환에 실패하면 임시 파일을 삭제하고 에러 메시지를 출력합니다.
        rm "${file}.tmp"
        echo "인코딩 변환 실패: $file"
    fi
done
```

**명령어 설명:**

  * `find`: `.msg`, `.srv`, `.action` 확장자를 가진 모든 파일을 찾습니다.
  * `iconv -f CP949 -t UTF-8`: `CP949` 형식(`-f` from)으로 된 파일을 `UTF-8` 형식(`-t` to)으로 변환합니다.
  * `while` 루프: 찾은 각 파일에 대해 변환 작업을 반복하고, 안전하게 원본 파일을 교체합니다.

#### **4. (선택 사항) 다른 소스 파일도 변환하기**

만약 위 단계를 거친 후에도 빌드 시 다른 파일에서 인코딩 에러가 발생한다면, 한글 주석 등이 포함된 C++(`.cpp`, `.hpp`)나 Python(`.py`) 소스 코드 파일도 인코딩이 변경되었을 수 있습니다. 그럴 경우 아래 명령어를 추가로 실행할 수 있습니다.

```bash
find ./src -type f \( -name "*.py" -o -name "*.cpp" -o -name "*.hpp" -o -name "CMakeLists.txt" -o -name "package.xml" \) -print0 | while IFS= read -r -d $'\0' file; do
    iconv -f CP949 -t UTF-8 "$file" -o "${file}.tmp"
    if [ $? -eq 0 ]; then
        mv "${file}.tmp" "$file"
        echo "인코딩 변환 완료: $file"
    else
        rm "${file}.tmp"
        echo "인코딩 변환 실패: $file"
    fi
done
```

#### **5. 이전 빌드 정리 및 재빌드**

인코딩 변환이 완료되었다면, 이전에 실패했던 빌드 결과물을 깨끗하게 지우고 처음부터 다시 빌드하는 것이 좋습니다.

```bash
# 작업 공간 최상위 폴더(~/ros2_ws)에서 실행
rm -rf build/ install/ log/
colcon build --symlink-install
```

이제 `colcon build`가 `UnicodeDecodeError` 없이 정상적으로 진행될 것입니다.

### 요약

이미 문제가 생긴 프로젝트는 Ubuntu의 `iconv` 명령어로 깨진 파일들의 인코딩을 UTF-8로 되돌린 후, `colcon build`를 다시 실행하면 해결할 수 있습니다. 가장 중요한 것은 **작업 전 `src` 폴더를 반드시 백업**하는 것입니다.
