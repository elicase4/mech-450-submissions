#!/bin/bash

LIST=list.txt
FILENAME=Project2

# Check for member one
STUDENT1=$1
if [ "$STUDENT1" != "" ]; then
    echo "First Project Member: $STUDENT1"
    FILENAME="${FILENAME}_${STUDENT1}"
else
    echo "First argument must be netID of student!"
    exit 1
fi

# Check for member two
STUDENT2=$2
if [ "$STUDENT2" != "" ]; then
    echo "Second Project Member: $STUDENT2"
    FILENAME="${FILENAME}_${STUDENT2}"
else
    echo "Second Project Member: N/A"
fi

# Check if all files exist
while read -r f; do
    if [[ ! -e $f ]]; then
        printf '%s is missing!\n' "$f"
        exit 1
    fi
done < ${LIST}

# Create archive
FILENAME="${FILENAME}.tar.gz"

rm -f ${FILENAME}
tar -czf ${FILENAME} -T ${LIST}
echo "Created submission archive ${FILENAME}"
