all: 
docs: doc
doc: 
	./rosdoc

.PHONY: doc

upload-local: doc
	rm -rf /var/www/pr.willowgarage.com/html/pr-docs/ros-packages
	cp -a doc /var/www/pr.willowgarage.com/html/pr-docs/ros-packages

clean: 
	rm -rf doc
