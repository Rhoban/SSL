#!/bin/sh
QRC=./www.qrc
echo '<!DOCTYPE RCC>' > $QRC
echo '<RCC version="1.0">' >> $QRC
echo '  <qresource>' >> $QRC

for a in $(find www -type f)
do
    # if this is not a folder
    if [ ! -d "$a" ]; then
        echo '      <file>'$a'</file>' >> $QRC
    fi
done

echo '  </qresource>' >> $QRC
echo '</RCC>' >> $QRC
