repo=../../

all: graph1.pdf graph2.pdf


clean:
	#rm -rf git-big-picture 
	rm -f graph*.pdf


graph1.pdf: git-big-picture
	git fetch origin --prune
	./git-big-picture/git-big-picture -o $@ -a $(repo) -M 

graph2.pdf: git-big-picture
	git fetch origin --prune
	./git-big-picture/git-big-picture -o $@ -a $(repo)

git-big-picture:
	git clone git://github.com/esc/git-big-picture.git



