# $Id: Makefile,v 1.3 2004/07/08 19:31:22 scaekenberghe Exp $

default:
	@echo Possible targets:
	@echo clean-openmcl --- remove all '*.dfsl' recursively
	@echo clean-lw --- remove all '*.nfasl' recursively
	@echo clean-emacs --- remove all '*~' recursively
	@echo clean --- all of the above

clean-openmcl:
	find . -name "*.dfsl" | xargs rm

clean-lw:
	find . -name "*.nfasl" | xargs rm

clean-emacs:
	find . -name "*~" | xargs rm

clean: clean-openmcl clean-lw clean-emacs

#
# This can obviously only be done by a specific person in a very specific context ;-)
#

PRJ=s-xml
ACCOUNT=scaekenberghe
CVSRT=:ext:$(ACCOUNT)@common-lisp.net:/project/$(PRJ)/cvsroot

release:
	rm -rf /tmp/$(PRJ) /tmp/public_html /tmp/$(PRJ).tgz /tmp/$(PRJ).tgz.asc
	cd /tmp; cvs -d$(CVSRT) export -r HEAD $(PRJ); cvs -d$(CVSRT) export -r HEAD public_html
	mv /tmp/public_html /tmp/$(PRJ)/doc
	cd /tmp; gnutar cvfz $(PRJ).tgz $(PRJ); gpg -a -b $(PRJ).tgz
	scp /tmp/$(PRJ).tgz $(ACCOUNT)@common-lisp.net:/project/$(PRJ)/public_html
	scp /tmp/$(PRJ).tgz.asc $(ACCOUNT)@common-lisp.net:/project/$(PRJ)/public_html
