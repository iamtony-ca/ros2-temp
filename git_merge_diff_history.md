# Q  
나는 github에 이미 어려 폴더와 파일이 올려져 있어.  
그리고 내가 local git으로 아애 별개의 폴더와 파일들이 있어. 이건 lcoal에서만 git commit만 여러번 했었어.  
  
내가 원하는 건 github remote repo에 내가 local git으로 관리했던 모든 폴더와 파일들을 업로드 하고 싶어. 이때 내가 기존에 local git으로 commit 했던 history들은 remote repo에 그대로 남기고 싶어.  
   
# A     
 Git 전문가로서 명확하게 설명해 드리겠습니다. 원격 GitHub 저장소와 로컬 저장소의 히스토리가 서로 다를 때, 로컬의 커밋 히스토리를 그대로 유지하면서 원격 저장소에 합치는 방법을 알려드리겠습니다.

이 작업의 핵심은 **관련 없는 히스토리(unrelated histories)를 병합**하는 것입니다. Git은 기본적으로 공통된 조상 커밋이 없는 두 브랜치를 병합하는 것을 허용하지 않지만, 특정 옵션을 통해 이를 가능하게 할 수 있습니다.

-----

### **작업 절차: 단계별 가이드**

로컬 저장소의 `main`(또는 `master`) 브랜치에 있는 내용을 원격 저장소의 `main` 브랜치에 합치는 것을 기준으로 설명하겠습니다.

#### **1단계: 로컬 Git 저장소에서 원격 저장소 추가**

먼저, 작업하려는 로컬 저장소 폴더로 이동한 후, 합치고 싶은 GitHub 원격 저장소를 `remote`로 등록해야 합니다.

```bash
# 1. 현재 작업 위치를 로컬 Git 저장소로 변경합니다.
cd /path/to/your/local/repo

# 2. GitHub 원격 저장소를 'origin'이라는 이름으로 추가합니다.
# <repository-url> 부분에 본인의 GitHub 저장소 URL을 입력하세요.
# 예: https://github.com/user/repo.git
git remote add origin <repository-url>
```

> 💡 **참고:** 만약 `fatal: remote origin already exists.` 라는 메시지가 나온다면, 이미 다른 원격 저장소가 'origin'이라는 이름으로 등록된 것입니다. `git remote -v`로 확인 후 `git remote remove origin`으로 삭제하거나, `git remote add github <repository-url>`처럼 다른 이름으로 추가할 수 있습니다.

-----

#### **2단계: 원격 저장소의 모든 정보 가져오기**

로컬에 원격 저장소의 모든 브랜치와 커밋 히스토리 정보를 가져옵니다. 이 단계에서는 아직 병합이 일어나지 않으므로 안전합니다.

```bash
git fetch origin
```

이 명령어를 실행하면 `origin/main` (또는 `origin/master`)과 같이 원격 저장소의 브랜치를 로컬에서 추적할 수 있게 됩니다.

-----

#### **3단계: 관련 없는 히스토리 병합 (가장 중요한 단계)**

이제 로컬 브랜치(`main`)와 원격 브랜치(`origin/main`)를 병합합니다. 이때 `--allow-unrelated-histories` 플래그를 사용하여 서로 다른 뿌리를 가진 두 히스토리를 합칠 수 있도록 허용해야 합니다.

```bash
# 원격 저장소의 main 브랜치를 현재 나의 main 브랜치로 병합합니다.
git merge origin/main --allow-unrelated-histories
```

이 명령을 실행하면 Git은 두 브랜치의 내용을 합치려고 시도합니다.

-----

#### **4단계: 충돌(Conflict) 해결**

만약 로컬 저장소와 원격 저장소에 **같은 이름의 파일**이 존재한다면, 거의 확실하게 \*\*병합 충돌(merge conflict)\*\*이 발생합니다.

1.  `git status` 명령어로 어떤 파일에서 충돌이 발생했는지 확인합니다.
2.  VS Code와 같은 코드 에디터로 충돌이 발생한 파일을 엽니다.
3.  파일을 열면 `<<<<<<<`, `=======`, `>>>>>>>` 와 같은 마커로 표시된 부분이 보일 겁니다. 이 부분을 직접 수정하여 최종적으로 남길 코드를 결정해야 합니다.
4.  충돌을 모두 해결했다면, 수정한 파일을 `add`하고 병합 커밋을 생성합니다.

<!-- end list -->

```bash
# 충돌을 해결한 파일을 stage 상태로 변경
git add .

# 병합을 완료하는 커밋 생성
# 이 명령을 실행하면 보통 자동으로 커밋 메시지가 작성된 편집기 창이 뜹니다.
# 그대로 저장하고 닫으면 됩니다.
git commit
```

> ⚠️ **중요:** 파일 충돌이 없다면 이 단계는 자동으로 넘어가며, 바로 병합 커밋이 생성됩니다.

-----

#### **5단계: 최종 결과 원격 저장소에 푸시**

이제 로컬 저장소에는 기존 로컬 커밋 히스토리와 원격 저장소의 히스토리가 모두 합쳐졌습니다. 이 최종 결과를 GitHub에 푸시(push)하면 됩니다.

```bash
git push origin main
```

이제 GitHub 저장소에 방문해 보세요. 원격 저장소에 있던 기존 파일들과 로컬에 있던 모든 파일 및 폴더가 함께 존재하며, 로컬에서 작업했던 모든 커밋 히스토리 또한 그대로 남아있는 것을 확인할 수 있습니다.

-----

### **요약 (명령어)**

```bash
# 1. 로컬 저장소 폴더로 이동
cd /path/to/your/local/repo

# 2. 원격 저장소 연결
git remote add origin <repository-url>

# 3. 원격 정보 가져오기
git fetch origin

# 4. 관련 없는 히스토리 병합
git merge origin/main --allow-unrelated-histories

# 5. (충돌 발생 시) 충돌 해결 후 커밋
git add .
git commit

# 6. GitHub에 최종본 푸시
git push origin main
```

이 방법이 질문하신 요구사항을 가장 정확하게 만족시키는 표준적인 절차입니다. 시작하기 전에 만약을 대비해 로컬 저장소 폴더를 복사하여 백업해두는 것을 추천합니다.
