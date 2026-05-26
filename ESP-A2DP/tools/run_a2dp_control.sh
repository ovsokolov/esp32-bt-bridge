#!/bin/sh
cd "$(dirname "$0")/.."

if [ -x /opt/anaconda3/bin/pythonw ] &&
    /opt/anaconda3/bin/pythonw -c "import tkinter, serial" >/dev/null 2>&1; then
    exec /opt/anaconda3/bin/pythonw tools/a2dp_control.py "$@"
fi

XCODE_PYTHON=/Applications/Xcode.app/Contents/Developer/usr/bin/python3
XCODE_PYTHON_APP=/Applications/Xcode.app/Contents/Developer/Library/Frameworks/Python3.framework/Versions/3.9/Resources/Python.app

if [ -x "$XCODE_PYTHON" ] &&
    [ -d "$XCODE_PYTHON_APP" ] &&
    "$XCODE_PYTHON" -c "import tkinter, serial" >/dev/null 2>&1; then
    open -n "$XCODE_PYTHON_APP" --args "$(pwd)/tools/a2dp_control.py" "$@"
    exit $?
fi

for python in \
    .venv/bin/python \
    /usr/bin/python3 \
    /Applications/Xcode.app/Contents/Developer/usr/bin/python3 \
    /opt/homebrew/bin/python3 \
    /opt/anaconda3/bin/python3 \
    python3
do
    if command -v "$python" >/dev/null 2>&1 &&
        "$python" -c "import tkinter, serial" >/dev/null 2>&1; then
        exec "$python" tools/a2dp_control.py "$@"
    fi
done

echo "No Python interpreter with tkinter and pyserial found." >&2
echo "Install pyserial for a Python that has tkinter, then retry." >&2
exit 1
