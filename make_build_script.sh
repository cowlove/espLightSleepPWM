#!/bin/bash 
BOARD=${BOARD:=$B}
BOARD=${BOARD:=esp32s3}

if [ "$BOARD" == "esp32" ]; then
	BOARD_OPTS=${BOARD_OPTS:=PartitionScheme=min_spiffs}
	PORT=${PORT:=/dev/ttyUSB0}
fi 
if [ "$BOARD" == "esp32c3" ] || [ $BOARD == "esp32c6" ] || [ $BOARD == "esp32s3" ]; then 
	BOARD_OPTS=${BOARD_OPTS:=PartitionScheme=min_spiffs,CDCOnBoot=cdc}
	PORT=${PORT:=/dev/ttyACM0}
fi

echo Building for ${BOARD}:${BOARD_OPTS} uploading to ${PORT}

# As of 3/30 8:30am this is the best working version of make_build

GIT_VERSION=`git describe --abbrev=6 --dirty --always`

cd "`dirname $0`"
SKETCH="`basename \`pwd\``"
BUILDDIR="/tmp/arduino/${SKETCH}/${BOARD}"
mkdir -p ${BUILDDIR}
TMP="/tmp/$$.txt"
arduino-cli compile -v -b esp32:esp32:${BOARD} --build-path ${BUILDDIR} \
  --board-options ${BOARD_OPTS} \
  --build-property compiler.cpp.extra_flags="-DGIT_VERSION=\"${GIT_VERSION}\"" \
   -u -p ${PORT}\
	 | tee "$TMP"

SKETCHDIR="$BUILDDIR"
SKETCHCPP="${SKETCHDIR}/sketch/${SKETCH}.ino.cpp"
OUT=./quick-${BOARD}.sh

# TODO: this is missing a link command        
COMPILE_CMD=`egrep "[-]o ${SKETCHCPP}.o" $TMP`
LINK_CMD1=`egrep " cr ${SKETCHDIR}/sketch/objs.a" $TMP`
LINK_CMD2=`egrep "[-]o ${SKETCHDIR}/${SKETCH}.ino.elf" $TMP`
LINK_CMD3=`grep esptool_py $TMP | grep -v ${PORT} | head -1 | tr '\n' ' '`
LINK_CMD4=`grep esptool_py $TMP | grep -v ${PORT} | tail -1 | tr '\n' ' '`
UPLOAD_CMD=`grep esptool_py $TMP | grep ${PORT} | tr '\n' ' '| sed s\|${PORT}\|\\\${PORT}\|g`

cat <<END > $OUT
#!/bin/bash 
PORT=\${PORT:=\$P}
PORT=\${PORT:=$PORT}

OPT=\$1; if [ "\$OPT" == "" ]; then OPT="-clwum"; fi
set -e
if [[ \$OPT == *c* ]]; then
	echo -n Compiling...
	echo '#include <Arduino.h>' > ${SKETCHCPP}
	echo "#line 1 \"`pwd`/${SKETCH}.ino\"" >> ${SKETCHCPP}
	cat "${SKETCH}.ino" >> ${SKETCHCPP}
	time $COMPILE_CMD
fi

if [[ \$OPT == *l* ]]; then
	echo Linking...
	$LINK_CMD1
	$LINK_CMD2
	$LINK_CMD3
	$LINK_CMD4  
fi

if [[ \$OPT == *u* ]]; then
	echo Uploading... 
	if [[ \$OPT == *w* ]]; then echo -n Waiting for \${PORT}...; while [ ! -e \${PORT} ]; do sleep .01; done; echo OK; fi;
	$UPLOAD_CMD 
fi

if [[ \$OPT == *m* ]]; then
	echo Monitoring...
	if [[ \$OPT == *w* ]]; then echo -n Waiting for \${PORT}...; while [ ! -e \${PORT} ]; do sleep .01; done; echo OK; fi;
	while sleep .01; do if [ -e \${PORT} ]; then stty -F \${PORT} 115200 raw -echo && cat \${PORT}; fi; done
fi;


END

chmod 755 $OUT
#rm -f $TMP
