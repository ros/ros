all:
	for dir in $(SUBDIRS); do cd $$dir && make && cd ..; done

clean: 
	for dir in $(SUBDIRS); do cd $$dir && make clean && cd ..; done
