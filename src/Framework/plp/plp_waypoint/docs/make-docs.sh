#!/bin/bash

cat head.html > wp-navigation-plp.html
markdown wp-navigation-plp.md >> wp-navigation-plp.html
cat footer.html >> wp-navigation-plp.html
