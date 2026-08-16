#!/usr/bin/env bash
# Build and test kinetics-core with nothing but a JDK. No network, no Gradle, no JUnit.
set -euo pipefail
cd "$(dirname "$0")/.."
export JAVA_HOME="${JAVA_HOME:-$HOME/jdks/jdk-25.0.4+7/Contents/Home}"
JAVAC="$JAVA_HOME/bin/javac"
JAVA="$JAVA_HOME/bin/java"

OUT=build/classes
rm -rf "$OUT" && mkdir -p "$OUT"

find core/src/main/java -name '*.java' > build/main-sources.txt
find core/src/test/java -name '*.java' > build/test-sources.txt

"$JAVAC" -d "$OUT" -Xlint:all,-serial @build/main-sources.txt
"$JAVAC" -d "$OUT" -cp "$OUT" -Xlint:all,-serial @build/test-sources.txt
cp -R core/src/main/resources/. "$OUT"/

echo "build ok: $(find "$OUT" -name '*.class' | wc -l | tr -d ' ') classes"
if [ "${1:-}" = "test" ]; then
  shift
  "$JAVA" -cp "$OUT" dev.lilkuzco.kinetics.test.TestMain "$@"
elif [ "${1:-}" = "golden-record" ]; then
  "$JAVA" -Dkinetics.golden.record=core/src/main/resources/golden-trajectories.txt -cp "$OUT" dev.lilkuzco.kinetics.test.TestMain golden
elif [ "${1:-}" = "audit" ]; then
  "$JAVA" -cp "$OUT" dev.lilkuzco.kinetics.constants.ScaleAudit SCALE-AUDIT.md
fi
