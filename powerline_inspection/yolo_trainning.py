from ultralytics import YOLO

def main():

    print('Starting trainnig')

    # load yolo 11
    model = YOLO('yolo11n.pt')

    data_yaml = '/home/leticia/drone_sim_ws/src/powerline_inspection_simulation/transmission_tower_dataset/data.yaml'

    
    # model configs 
    resultados = model.train(
        data=data_yaml, 
        epochs=50, 
        imgsz=640, 
        batch=4,
        device=0,
        plots=True,
        project='tower_identification', 
        name='yolo11_test'
    )


    print('Trainning completed')


if __name__ == '__main__':
    main()