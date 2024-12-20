WORKSPACE=../../engine.code-workspace
cleaned_temp_file=$(mktemp)
json5 "$WORKSPACE" -s 2 -o "$cleaned_temp_file"
yaml_temp_file=$(mktemp)
yq eval -P "$cleaned_temp_file" > "$yaml_temp_file"
merged_temp_file=$(mktemp)
yq eval-all 'select(fileIndex == 0) * select(fileIndex == 1)' \
  "$yaml_temp_file" engine-workspace.yaml > "$merged_temp_file" && \
  mv "$merged_temp_file" engine-workspace.yaml
rm "$yaml_temp_file"
rm "$cleaned_temp_file"
