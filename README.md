# README #

This is a collection of ROS packages maintained by [Maeve Automation](https://maeveautomation.com) for
experimentation purposes. No warranty or guarantee is made to the usefulness or correctness of the code.

Each package contains its own README with details about its contents.

If using the code, be aware that different licenses may apply to different parts
of the code.

## Cloning ##

See the [Overview](https://bitbucket.org/maeveautomation/open_maeve/overview)
page for links to clone this repository.

## Releases ##

Each release of the codebase is tagged with `release-X` where 'X' is the
release number. Releases represent functionality that is in some sense complete
and tested. To get a release 'X', clone the repo, then:

    hg pull && hg up release-X

Revisions of the codebase between releases may include new functionality and
fixes but may also include incomplete or broken code, so caveat emptor.

## Collaboration ##

Collaboration is welcome using [feature branches](https://www.atlassian.com/git/tutorials/comparing-workflows#feature-branch-workflow) or [forks](https://www.atlassian.com/git/tutorials/comparing-workflows#forking-workflow).

## Citing ##

If any code in these libraries is useful for published work, a citation is appreciated. The below bibtex may be used:

    @misc{maeve-development-libraries,
        author          = {Jeffrey Kane Johnson},
        title           = {{Maeve Automation Development Libraries}},
        howpublished    = {\url{https://github.com/togaen/open_maeve}},
        year            = {2017},
        institution     = {{Maeve Automation}}
    }

## Data Sets ##

Maeve maintains and distributes data sets used and recorded during the development of the libraries. A list of data sets can be found on the [Data Sets](https://maeveautomation.com/data-sets/) web page.
