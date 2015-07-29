#!/bin/bash
#
# Simple script to make a d-collide source distribution.
# Usage
#   ./make_source_release <path_to_d-collide_sources> <release_name>


SOURCE_DIRECTORY="$1"
RELEASE_NAME="$2"

NON_SVN_MODELS="bunny.ply"
NON_SVN_MODELS="$NON_SVN_MODELS bunny_lower_res2.ply"
NON_SVN_MODELS="$NON_SVN_MODELS bunny_lower_res3.ply"
NON_SVN_MODELS="$NON_SVN_MODELS bunny_lower_res4.ply"


# export name without .tar.gz
EXPORT_DIRECTORY="d-collide-$RELEASE_NAME"

EXPORT_ARCHIVE="$EXPORT_DIRECTORY".tar.gz

function print_usage() {
    echo "Usage: $0 <path_to_dcollide_sources> <release_name>"
}

if [ ! -d "$SOURCE_DIRECTORY" ]; then
    if [ -z "$SOURCE_DIRECTORY" ]; then
        print_usage
    else
        echo "Source directory $SOURCE_DIRECTORY does not exist"
    fi
    exit 1
fi

if [ ! -d "$SOURCE_DIRECTORY/testapp" ]; then
    echo "$SOURCE_DIRECTORY does not seem to contain d-collide sources"
    exit 1
fi

if [ -d "$EXPORT_DIRECTORY" ]; then
    echo "Export directory $EXPORT_DIRECTORY already exists!"
    exit 1
fi

if [ -e "$EXPORT_ARCHIVE" ]; then
    echo "$EXPORT_ARCHIVE already exists!"
    exit 1
fi

svn export "$SOURCE_DIRECTORY" "$EXPORT_DIRECTORY" > /dev/null
if [ "$?" != "0" ]; then
    echo "svn export failed."
    exit 1
fi
for model in $NON_SVN_MODELS; do
    file_path="testapp/resources/models/$model"
    if [ ! -r "$SOURCE_DIRECTORY/$file_path" ]; then
        echo "non-svn model $model not found"
        exit 1
    fi
    cp "$SOURCE_DIRECTORY/$file_path" "$EXPORT_DIRECTORY/$file_path" || exit 1
done

tar czf "$EXPORT_ARCHIVE" "$EXPORT_DIRECTORY"
if [ "$?" != "0" ]; then
    echo "tar czvf failed."
    exit 1
fi

rm -rf "$EXPORT_DIRECTORY"

echo "Distribution completed: $EXPORT_ARCHIVE"

# vim: et sw=4 ts=4
