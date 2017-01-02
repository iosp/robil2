#!/bin/bash
NAME=$(pwd | tr / \\n | tail -n 2 | head -n 1)
echo Creating $NAME.html from $NAME.md

cat head.html > $NAME.html
markdown $NAME.md >> $NAME.html
cat footer.html >> $NAME.html
