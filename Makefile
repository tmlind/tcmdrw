all: clean
	$(CROSS_COMPILE)gcc -Wall -static -o tcmdrw tcmdrw.c
	#$(CROSS_COMPILE)strip tcmdrw

clean:
	rm -f tcmdrw *~
