#!/bin/sh

set -e

if [ ! -f samples/README ] ; then
    echo "ERROR: Run 'git submodule update --init test/samples' first."
    exit 1
fi

rm -rf decoded
mkdir -p decoded/dmix_0 decoded/dmix_2 decoded/dmix_6 decoded/mono
for i in samples/*.dtshd ; do
    ../dcadec -b -q $i decoded/dmix_0/$(basename $i .dtshd).wav
    ../dcadec -b -q -2 $i decoded/dmix_2/$(basename $i .dtshd).wav
    ../dcadec -b -q -6 $i decoded/dmix_6/$(basename $i .dtshd).wav
    ../dcadec -m -q $i decoded/mono/$(basename $i .dtshd)_%s.wav
done

if [ "$1" = "--update" ] ; then
    sha1sum -b decoded/dmix_0/*.wav decoded/dmix_2/*.wav decoded/dmix_6/*.wav > checksum.txt
    sha1sum -b samples/reference/xll_*.wav | sed 's|samples/reference|decoded/mono|' >> checksum.txt
    for i in decoded/mono/core_*.wav decoded/mono/x96_*.wav decoded/mono/xbr_*.wav decoded/mono/xch_*.wav decoded/mono/xxch_*.wav ; do
        ./stddev $i samples/reference/$(basename $i) ?
    done > stddev.txt
else
    sha1sum -c checksum.txt
    for i in decoded/mono/core_*.wav decoded/mono/x96_*.wav decoded/mono/xbr_*.wav decoded/mono/xch_*.wav decoded/mono/xxch_*.wav ; do
        ./stddev $i samples/reference/$(basename $i) $(grep -F $i stddev.txt | cut -d ' ' -f 2)
    done
fi
