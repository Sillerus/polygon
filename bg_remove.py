import numpy as np
import os
from PIL import Image


# Пороговые значения указываю по умолчанию в функции
def rem_bg(image_path, r_range=(215, 255), g_range=(140, 240), b_max=120):
    # Открываем изображение при помощи Pillow
    image = Image.open(image_path).convert("RGBA")

    # Преобразуем изображение в массив numpy типа
    data = np.array(image)

    # Создание маски с учетом пороговых значений (для желтого фона)
    mask = (
            (data[:, :, 0] >= r_range[0]) & (data[:, :, 0] <= r_range[1]) &  # Красный канал в заданном диапазоне
            (data[:, :, 1] >= g_range[0]) & (data[:, :, 1] <= g_range[1]) &  # Зелёный канал в заданном диапазоне
            (data[:, :, 2] <= b_max)  # Синий канал ниже заданного максимума
    )

    # Установка альфа-канала, ставим 0, чтобы нужные пиксели были прозрачными
    data[mask, 3] = 0

    # Конвертируем массив обратно в изображение с помощью Pillow
    result_image = Image.fromarray(data, mode="RGBA")
    filename, file_extension = os.path.splitext(image_path)
    # Сохраняем результат в формате png
    result_image.save(filename + '_without_bg.png')


# Инициализация функции
rem_bg('catsf.png')