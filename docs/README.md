This folder represents the Github Pages site that is used to host Stabilizer's user guide.

The site is hosted with Jekyll and utilizes the "Just the Docs" theme.

To run locally:
1. Install Ruby
1. Install [Jekyll](https://jekyllrb.com/)
1. Install [Bundler](https://bundler.io/)
1. From this directory:
```
bundle install
bundle exec jekyll serve
```
1. Navigate to `localhost:4000` in a web browser

Note: Some of the links in the docs rely on Cargo's documentation. To make all links work locally, run:
```
cargo doc --bins
cp -r target/thumbv7em-none-eabihf/doc docs/firmware
```
