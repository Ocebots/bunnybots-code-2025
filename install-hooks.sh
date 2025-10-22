#!/usr/bin/env zsh

echo "#!/usr/bin/env zsh\n\n./gradlew build\n./gradlew spotlessCheck" > .git/hooks/pre-commit
chmod +x .git/hooks/pre-commit
