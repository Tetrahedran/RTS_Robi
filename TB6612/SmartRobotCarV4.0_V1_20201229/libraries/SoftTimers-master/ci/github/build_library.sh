# Any commands which fail will cause the shell script to exit immediately
set -e

# Validate github's environment
if [ "$GITHUB_WORKSPACE" = "" ]; then
  echo "Please define 'GITHUB_WORKSPACE' environment variable.";
  exit 1;
fi

unset CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH;$GITHUB_WORKSPACE/third_parties/googletest/install"
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH;$GITHUB_WORKSPACE/third_parties/RapidAssist/install"
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH;$GITHUB_WORKSPACE/third_parties/win32Arduino/install"

echo ============================================================================
echo Generating SoftTimers...
echo ============================================================================
cd $GITHUB_WORKSPACE
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" -DSOFTTIMERS_BUILD_EXAMPLES=ON ..

echo ============================================================================
echo Compiling SoftTimers...
echo ============================================================================
cmake --build .
echo
