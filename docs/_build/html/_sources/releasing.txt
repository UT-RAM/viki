.. _`Releasing`:

Releasing
=========

Workflow for releasing a new version:

    #. Checkout to a new release branch based on the commit that the release needs to be based on
    #. Change the VERSION file by removing -dev from the version and possibly changing the name
    #. Run the bumpversion.py script, which adds the right licensing to the files, based on the version provided in the VERSION file. **Make this a seperate commit**. If anything goes wrong, it is easy to revert this.
    #. Commit and push release branch to github
    #. Submit a pull request on GitHub and discuss
    #. Create a new release using GitHub (marks the commit as release)
    #. On the dev branch, make sure to change the version to <target version>-dev

Possibly, a lot of this can be automated. Since this is not that much work, this is not done at the moment,
but would be not that difficult to do.

Hotfixing
---------

When a hotfix needs to be made, make sure to checkout to the commit **before** the bumpversion has run. Based on that commit

    #. Create a new hotfix branch
    #. Make your changes and commit (only the neccessary changes)
    #. Follow the release step 2-6 to publish this hotfix immediately
