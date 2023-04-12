function ed_object_models_setup {
    local model_path=$(rospack find my_ed_object_models)/models

    local model_paths=($(find $model_path -type f -name '*model*.sdf' | xargs dirname | xargs dirname))
    local unique_model_paths=$(printf "%s\n" "${model_paths[@]}" | sort -ru | tr '\n' ' ')
    local model_paths_string=""
    for dir in $unique_model_paths
    do
        if [[ "$dir" == "$model_path" ]]
        then
            # You want to have the main model path at the beginning
            continue
        fi
        model_paths_string="$dir${model_paths_string:+:${model_paths_string}}"
    done

    model_paths_string="$model_path${model_paths_string:+:${model_paths_string}}"

    export ED_MODEL_PATH=$model_path${ED_MODEL_PATH:+:${ED_MODEL_PATH}}

    export GAZEBO_MODEL_PATH=$model_paths_string${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}
    export GAZEBO_RESOURCE_PATH=$model_paths_string${GAZEBO_RESOURCE_PATH:+:${GAZEBO_RESOURCE_PATH}}
}

ed_object_models_setup
