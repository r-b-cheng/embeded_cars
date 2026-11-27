#!/usr/bin/env bash
# Offline build with stock Android SDK tools and a local Android debug certificate.
set -euo pipefail
cd "$(dirname "$0")"
sdk="${ANDROID_HOME:-$HOME/Android/Sdk}"
bt="$sdk/build-tools/${BUILD_TOOLS:-35.0.0}"
jar="$sdk/platforms/android-35/android.jar"
key="${PATROL_KEYSTORE:-$HOME/.android/debug.keystore}"
if [[ ! -f "$key" ]]; then
    mkdir -p "$(dirname "$key")"
    keytool -genkeypair -keystore "$key" -storepass android -keypass android \
        -alias androiddebugkey -keyalg RSA -keysize 2048 -validity 10000 -dname "CN=Android Debug"
fi
mkdir -p build/classes build/dex ../dist
"$bt/aapt" package -f -A assets -S res -M AndroidManifest.xml -I "$jar" -F build/base.apk
javac --release 8 -encoding UTF-8 -cp "$jar" -d build/classes src/org/embeddedcars/patrol/*.java
jar cf build/classes.jar -C build/classes .
"$bt/d8" --lib "$jar" --min-api 24 --output build/dex build/classes.jar
(cd build/dex && "$bt/aapt" add ../base.apk classes.dex)
"$bt/zipalign" -f 4 build/base.apk build/aligned.apk
"$bt/apksigner" sign --ks "$key" --ks-pass pass:android --ks-key-alias androiddebugkey \
    --v4-signing-enabled true --out ../dist/patrol-debug.apk build/aligned.apk
"$bt/apksigner" verify ../dist/patrol-debug.apk
