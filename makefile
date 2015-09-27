#
## Gonk Makefile
#

clean:	tilde
	find ./ -name core | xargs rm -f

tilde:
	find ./ -name \*~ | xargs rm -f

commit: clean
	git pull
	git add .
	git commit -a
	git push

dollars:
	sloccount .
