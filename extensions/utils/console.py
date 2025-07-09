def print_progress_bar(current: int, total: int, bar_width: int = 20):
    percent = int((current / total) * 100)
    bar_filled = int(bar_width * current / total)
    bar = "█" * bar_filled + "-" * (bar_width - bar_filled)
    print(f"[{percent:>3}%/100%] {bar} [{current}/{total}]")