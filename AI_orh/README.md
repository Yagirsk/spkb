### Инструкция

1. Для начала нужно установить ollama https://ollama.com/download
2. В самой ollama нужно установить модель qwen 3.6 или `ollama pull qwen3.6` в powershell (занимает примерно 23 ГБ)
3. Закидываем программу `AI_organaizer.py` в папку например C:\Organizer
4. В powershell по очереди: 
    
    `cd C:\Organizer`

    `pip install customtkinter pillow ollama`

    `python AI_organaizer.py`
5. Готово

### Программа `AI_organaizer.py`:
```py
import json
from pathlib import Path
from typing import Tuple, Dict
import tkinter as tk
from tkinter import filedialog, messagebox
import customtkinter as ctk
import threading
from PIL import Image, ImageTk

OLLAMA_MODEL = "qwen3.6"

PHOTOS_DIR = Path("organizer_photos")
PHOTOS_DIR.mkdir(exist_ok=True)

ROWS, COLS = 8, 8
TABLE_FILE = Path("organizer_table.json")


class OrganizerApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title(f"Органайзер — {OLLAMA_MODEL}")
        self.geometry("1420x920")
        self.resizable(True, True)

        self.items: list[Dict] = self.load_table()
        self.cell_buttons = []
        self.is_analyzing = False

        self.setup_ui()
        self.refresh_grid()

    def setup_ui(self):
        top_frame = ctk.CTkFrame(self, height=80)
        top_frame.pack(fill="x", padx=15, pady=10)

        self.title_label = ctk.CTkLabel(top_frame, text=f"🗃️ Органайзер — {OLLAMA_MODEL}", 
                                       font=ctk.CTkFont(size=26, weight="bold"))
        self.title_label.pack(side="left", padx=10)

        self.add_button = ctk.CTkButton(top_frame, text="➕ Добавить ячейки (можно несколько)", 
                                       height=45, font=ctk.CTkFont(size=16), 
                                       command=self.add_new_cells)
        self.add_button.pack(side="right", padx=15)

        # Сетка 8x8
        grid_frame = ctk.CTkFrame(self)
        grid_frame.pack(fill="both", expand=True, padx=15, pady=10)

        self.cell_buttons = [[None for _ in range(COLS)] for _ in range(ROWS)]

        for r in range(ROWS):
            for c in range(COLS):
                btn = ctk.CTkButton(
                    grid_frame, 
                    text="—",
                    width=158,
                    height=98,
                    fg_color="#2b2b2b",
                    hover_color="#363636",
                    font=ctk.CTkFont(size=12),
                    corner_radius=10,
                    command=lambda row=r, col=c: self.show_cell_details(row, col)
                )
                btn.grid(row=r, column=c, padx=6, pady=6, sticky="nsew")
                self.cell_buttons[r][c] = btn

        for i in range(ROWS):
            grid_frame.rowconfigure(i, weight=1)
        for i in range(COLS):
            grid_frame.columnconfigure(i, weight=1)

    def analyze_with_ollama(self, image_path: str) -> Tuple[str, str]:
        try:
            import ollama
            response = ollama.chat(
                model=OLLAMA_MODEL,
                messages=[{
                    'role': 'user',
                    'content': """Ты — эксперт по электронным компонентам и техническим лабораториям.
На фото — содержимое одной ячейки органайзера.

Опиши максимально точно:

1. Короткое информативное название ячейки (3–7 слов).

2. Содержимое в формате списка:
• Группа деталей: примерное количество
• Другая группа: примерное количество

Используй маркеры • и диапазоны (например 10–15 шт.).

Ответ строго в JSON:
{
  "cell_name": "Название ячейки",
  "description": "• Группа1: количество\n• Группа2: количество\n..."
}""",
                    'images': [image_path]
                }]
            )

            content = response['message']['content'].strip()
            if "```" in content:
                content = content.split("```json")[-1].split("```")[0].strip() if "```json" in content else content.split("```")[1].strip()

            data = json.loads(content)
            return data.get("cell_name", "Без названия").strip(), data.get("description", "").strip()

        except Exception as e:
            return "Ошибка распознавания", str(e)[:150]

    def add_new_cells(self):
        if self.is_analyzing:
            return

        files = filedialog.askopenfilenames(
            title="Выберите фото ячеек (можно несколько)",
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.webp *.bmp")]
        )
        if not files:
            return

        threading.Thread(target=self.process_multiple_files, args=(files,), daemon=True).start()

    def process_multiple_files(self, files):
        self.is_analyzing = True
        self.after(0, lambda: self.add_button.configure(state="disabled", text="Анализ..."))

        for i, file_path in enumerate(files, 1):
            self.after(0, lambda n=i, total=len(files), fname=Path(file_path).name: 
                      self.title_label.configure(text=f"Анализ {n}/{total}: {fname}"))

            name, desc = self.analyze_with_ollama(file_path)

            # Копируем фото
            try:
                new_path = PHOTOS_DIR / Path(file_path).name
                if not new_path.exists():
                    import shutil
                    shutil.copy(file_path, new_path)
            except:
                pass

            self.items.append({
                "name": name,
                "description": desc,
                "photo": Path(file_path).name
            })

        self.after(0, self.finish_batch)

    def finish_batch(self):
        self.refresh_grid()
        self.title_label.configure(text=f"Органайзер — {OLLAMA_MODEL}")
        self.add_button.configure(state="normal", text="➕ Добавить ячейки (можно несколько)")
        self.is_analyzing = False
        messagebox.showinfo("Готово", f"Успешно добавлено {len(self.items)} ячеек")

    def show_cell_details(self, row: int, col: int):
        item = next((it for it in self.items if it.get("row") == row and it.get("col") == col), None)
        if not item:
            return

        win = ctk.CTkToplevel(self)
        win.title(item.get("name", "Ячейка"))
        win.geometry("760x740")

        # Фото
        try:
            img_path = PHOTOS_DIR / item.get("photo", "")
            if img_path.exists():
                img = Image.open(img_path)
                img.thumbnail((340, 340))
                photo_img = ImageTk.PhotoImage(img)
                label = ctk.CTkLabel(win, image=photo_img, text="")
                label.image = photo_img
                label.pack(pady=10)
        except:
            ctk.CTkLabel(win, text="Фото не найдено", text_color="gray").pack(pady=10)

        # Название
        ctk.CTkLabel(win, text="Название ячейки", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10,5))
        name_entry = ctk.CTkEntry(win, width=700)
        name_entry.insert(0, item.get("name", ""))
        name_entry.pack(pady=5, padx=20)

        # Описание
        ctk.CTkLabel(win, text="Описание содержимого", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10,5))
        desc_text = ctk.CTkTextbox(win, width=700, height=280)
        desc_text.insert("0.0", item.get("description", ""))
        desc_text.pack(pady=5, padx=20)

        # Кнопки
        btn_frame = ctk.CTkFrame(win)
        btn_frame.pack(pady=20)
        ctk.CTkButton(btn_frame, text="Сохранить", width=200, height=40,
                      command=lambda: self.save_changes(item, name_entry, desc_text, win)).pack(side="left", padx=10)
        ctk.CTkButton(btn_frame, text="Удалить", width=200, height=40, fg_color="#c42b1c",
                      command=lambda: self.delete_cell(item, win)).pack(side="left", padx=10)

    def save_changes(self, item, name_entry, desc_text, win):
        item["name"] = name_entry.get().strip()
        item["description"] = desc_text.get("0.0", "end").strip()
        self.refresh_grid()
        win.destroy()

    def delete_cell(self, item, win):
        if messagebox.askyesno("Удалить?", f"Удалить эту ячейку?\n\n{item.get('name')}", icon="warning"):
            self.items.remove(item)
            self.refresh_grid()
            win.destroy()

    def load_table(self):
        if TABLE_FILE.exists():
            try:
                with open(TABLE_FILE, "r", encoding="utf-8") as f:
                    return json.load(f)
            except:
                return []
        return []

    def save_table(self):
        try:
            with open(TABLE_FILE, "w", encoding="utf-8") as f:
                json.dump(self.items, f, ensure_ascii=False, indent=2)
        except:
            pass

    def refresh_grid(self):
        self.items.sort(key=lambda x: x.get("name", "").lower())

        for i, item in enumerate(self.items):
            item["row"] = i // COLS
            item["col"] = i % COLS

        for r in range(ROWS):
            for c in range(COLS):
                self.cell_buttons[r][c].configure(text="—", fg_color="#2b2b2b")

        for item in self.items:
            r, c = item.get("row"), item.get("col")
            if r is None or c is None:
                continue
            name = item.get("name", "—")
            display = name if len(name) <= 28 else name[:28] + "\n" + name[28:56]
            self.cell_buttons[r][c].configure(text=display, fg_color="#1f6aa5")

        self.save_table()


if __name__ == "__main__":
    app = OrganizerApp()
    app.mainloop()
```
