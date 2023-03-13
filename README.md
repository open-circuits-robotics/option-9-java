# option-9-java

## Building
To build the code use gradlew
```bash
./gradlew
```

## Git Push/Pull

Before you make any changes it is helpful to have the latest version
```bash
git pull https://github.com/open-circuits-robotics/option-9-java.git
```

1. Remove the build folder
Un*x (Linux/MacOS)
```bash
rm -rf build
```
Windows:
```cmd
rmdir build
```

2. Add files to git

```bash
git add *
```

3. Commit to local repo
```bash
git commit -m "{commit message}"
```

4. Upload to github
```bash
git push --set-upstream https://github.com/open-circuits-robotics/option-9-java.git main
```

