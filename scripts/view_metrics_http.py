#!/usr/bin/env python3
"""
简单的 HTTP 服务器，用于在浏览器中查看性能指标图表
适用于 Docker 环境
"""

import http.server
import socketserver
import os
import sys

PORT = 8000
METRICS_DIR = "/tmp/ego_planner_metrics"

class MetricsHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=METRICS_DIR, **kwargs)
    
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Expires', '0')
        super().end_headers()

def main():
    if not os.path.exists(METRICS_DIR):
        print(f"错误: 指标目录不存在: {METRICS_DIR}")
        print("请先运行: python3 scripts/visualize_metrics.py --save")
        sys.exit(1)
    
    print("=" * 60)
    print(f"性能指标 HTTP 服务器")
    print("=" * 60)
    print(f"服务目录: {METRICS_DIR}")
    print(f"监听端口: {PORT}")
    print()
    print("可用文件:")
    for f in os.listdir(METRICS_DIR):
        fpath = os.path.join(METRICS_DIR, f)
        size = os.path.getsize(fpath) / 1024
        print(f"  - {f} ({size:.1f} KB)")
    print()
    print("访问方式:")
    print(f"  浏览器打开: http://localhost:{PORT}/metrics_plot.png")
    print(f"  CSV 数据:   http://localhost:{PORT}/<csv_filename>")
    print()
    print("按 Ctrl+C 停止服务器")
    print("=" * 60)
    
    try:
        with socketserver.TCPServer(("", PORT), MetricsHTTPRequestHandler) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n\n服务器已停止")
    except OSError as e:
        if e.errno == 98:
            print(f"\n错误: 端口 {PORT} 已被占用")
            print(f"请尝试关闭其他服务或修改脚本中的 PORT 变量")
        else:
            raise

if __name__ == '__main__':
    main()
