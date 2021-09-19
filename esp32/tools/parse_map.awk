# Parse ELF map file memory map
#
# Collect all files from memory map and sum the size of each object file.
# Print summary information for each file from large to small.
# File paths can be filtered by EXCLUDE and INCLUDE patterns
# Sections can be filtered by INCLUDE_SECTION and EXCLUDE_SECTION patterns.
#
# Handle two cases:
# 1. Symbol name is short and appears in the same line as the file name
# 2. Symbol name is long so it appears one line before the file name
#
# Symbol name and address are ignored, only symbol size and file name
# are taken into account
#
# Usage example:
#
# gawk -f parse_map.awk -v INCLUDE="/lib/" image.map
#
# This will filter only object files in /lib/ path and sum their sizes

BEGIN{
    enable=0;
}

/Linker script and memory map/{
    enable = 1;
}

enable && ($1~/[.].*/){
    section = $1
}

enable && INCLUDE_SECTION != "" && (section!~INCLUDE_SECTION){
    next
}

enable && EXCLUDE_SECTION != "" && (section~EXCLUDE_SECTION){
    next
}

enable && ($1~/0x.*/) && ($2~/0x.*/) && ($3~/.*[.].$/){
    sum($2, $3)
}

enable && ($2~/0x.*/) && ($3~/0x.*/) && ($4~/.*[.].$/){
    sum($3, $4)
}

END{
    print "==============================="
    printf("TOTAL SIZE = %d (0x%X)\n", total_size, total_size)
    print "==============================="
    printf("%-10s %-10s %-10s\n", "Size", "Size (hex)", " Filename")
    printf("%-10s %-10s %-10s\n", "====", "==========", " ========")

    for (file in files){
        sizes[files[file]] = sizes[files[file]] " " file
    }
    n = asort(files, sorted_sizes)
    for (i = n; i >= 1; i--){
        name = sizes[sorted_sizes[i]]
        size = sorted_sizes[i]
        printf("%-10d %-10s %-10s\n", size, sprintf("0x%X",size), name)
    }
}

function sum(size_str, filename){
    if (INCLUDE != "" && (filename !~ INCLUDE)) next
    if (EXCLUDE != "" && (filename ~ EXCLUDE)) next
    size = strtonum(size_str)
    if (size > 0){
        files[filename] += size
        total_size += size
    }
}
