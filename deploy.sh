#!/bin/sh
set -eu

cd "$(dirname "$0")"
deploy_dir=$(mktemp -d)
trap 'rm -rf "$deploy_dir"' EXIT
trap 'exit 1' HUP INT TERM

mkdir "$deploy_dir/public"
cp wiring-academy.html "$deploy_dir/public/index.html"
cat > "$deploy_dir/wrangler.toml" <<'CONFIG'
name = "wiring-academy"
compatibility_date = "2026-11-16"
workers_dev = false
preview_urls = false
routes = [{ pattern = "wiring-academy.tech619.com", custom_domain = true }]

[assets]
directory = "./public"
CONFIG

wrangler deploy --config "$deploy_dir/wrangler.toml"
