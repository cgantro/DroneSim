# 테스트 안내

테스트는 기능별 폴더에 모아 두었다. 로봇 수치와 이동은 `robotics`, 물리 연동은 `integration`, 전체 창 실행은 `ViewerSmoke`를 확인한다.

```powershell
ctest --test-dir build-ninja-debug -L robotics --output-on-failure
ctest --test-dir build-ninja-debug -L integration --output-on-failure
```

특정 테스트 하나만 실행하려면 끝에 `-R "테스트 이름"`을 붙인다. 이 테스트들은 주요 동작을 확인하는 표본이며 모든 자세나 충돌 상황을 보장하지 않는다.
