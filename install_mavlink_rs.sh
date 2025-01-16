

#update and install build essential
sudo apt update
sudo apt install build-essential pkg-config libssl-dev
sudo apt install cargo
rustup update

# clone the the git repo for mavlink-rs

git clone https://github.com/elmopl/mavlink-rs.git

# build the crate

cd mavlink-rs
cargo build --release

#include compiled library with rustc

#rustc -L path/to/compiled/library your_program.rs --extern mavlink=path/to/compiled/mavlink.rlib
