# Docs Usage

Glad you're here! This means you want to be able to see and add documentation for our GNC flight code. Here are the things you need to know: 
## Project layout

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.

Whenever you add or reorganize files, you have to go into `mkdocs.yml` and edit the structure to reflect your edits. If you're just changing the contents of files, no need to do anything here. 
## Writing Documentation

To write documentation, I'd highly recommend installing [Obsidian](https://obsidian.md/) and using it in our repo's `docs` folder to allow you to write to these Markdown files very quickly. This way, as you do your work, you can write documentation to record your decisions more easily. 

## Building the Site
* `pip install mkdocs-shadcn mkdocs-terminal mkdocs-mermaid2-plugin` Install themes/plugins.
* `mkdocs serve` - Start the live-reloading docs server.
* `mkdocs build` - Build the documentation site.
* `mkdocs -h` - Print help message and exit.

For full documentation visit [mkdocs.org](https://www.mkdocs.org) :)
-Lundeen