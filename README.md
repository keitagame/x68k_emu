# x68core — X68000 libretro コア (C / WebAssembly)

WebX68k ホスト (`core.js`) と **同じインターフェース** で、`px68k_libretro.js`（Emscripten glue）の
`px68k_libretro.wasm` と差し替えて使える、X68000 エミュレータコアの独自実装です。
CPU・バス・各 I/O・映像・音源はすべてこのリポジトリ内の C コードで、外部ライブラリは使っていません
(Musashi は CPU の差分テストにだけ使用し、配布物には含みません)。

## 使い方
1. `pip install ziglang` ののち `./build.sh`（`build/px68k_libretro.wasm` ができます）
2. ホスト側の `px68k_libretro.wasm` をこのファイルに置き換えるだけ（glue の `px68k_libretro.js` は**元のまま**）
3. ROM は従来どおり `/system/keropi/iplrom.dat`（128KB、`iplromxv.dat` 等も可）、`cgrom.dat`、任意で `sram.dat`（16KB）

## 実装状況
| 区分 | 内容 | 状態 |
|---|---|---|
| CPU | MC68000 全命令・例外・割り込み（自作、Musashi と 600 万命令の差分テストで一致） | ○ |
| メモリ | RAM 1–12MB(`px68k_ramsize`)、GVRAM/TVRAM、SRAM(書込保護)、CGROM/IPL、バスエラー | ○ |
| 映像 | CRTC、テキスト(同時書込/マスク/ラスタコピー)、グラフィック 16/256/65536色・1024 面、スプライト/BG、優先順位、ラスタ単位描画 | ○（特殊優先・半透明は未実装） |
| 音 | YM2151(FM 8ch・LFO・ノイズ・タイマ)、MSM6258 ADPCM(DMA ch3 連動)、44.1kHz ミックス | ○ |
| 割り込み | MFP68901(タイマ A–D、GPIP、キーボード)、IOC、DMAC、SCC | ○ |
| DMAC | HD63450 4ch、連鎖転送、外部要求 | ○ |
| FDD | uPD72065 + XDF / DIM / D88、書き込み(ファイルへ直接反映)、フォーマット | ○ |
| HDD | SASI (.hdf, 256B/sector) 1台 | ○（SCSI は未実装） |
| 入力 | キーボード(make/break)、マウス(SCC)、ジョイスティック 2 ポート | ○ |
| その他 | RTC(ホスト時刻)、シリアル(SCC ch.A)ブリッジ、セーブステート(約 3MB) | ○ |

## ホスト interface の対応
`retro_*` 一式と、`core.js` が呼ぶ `webx68k_*` / `get_*` / `clear_*` はすべて実装しています。
- 実装: tvram_data, text_dot_x/y, text_scroll_x/y, send_key_make, peek8/16, mem_read/write, sram_read/write,
  configured_ram_size, joystick_read, fdd_insert/eject, serial_*, get_fdd_*, get_sasi_*, get_mouse_*
- **スタブ(常に 0 / no-op)**: `webx68k_hostfs_*`（Human68k ホストFSドライバ連携）、`get_scsi_*`（SCSI なし）
- **推測実装**: `webx68k_keybuf_write_pointer`（IOCS ワーク 0x0812 のワードを返す。元コアの仕様が不明なため要確認）

## テスト（`test/run_all.sh`）
本物の glue(`px68k_libretro.js`) 上で Node から実行: 割り込み/描画/FM 音程(≈490Hz)、FDC+DMAC によるブートセクタ読込と実行、
キー make/break と joystick、SASI 読込、セーブステートの決定性、RTC。
`test/fetch_musashi_and_difftest.sh` で CPU の差分テスト。

## 重要な注意（未検証事項）
- **実機 IPL ROM / 実ディスクでの動作確認はできていません**（ROM は著作物のため手元になく、自作の最小 IPL でのみ検証）。
  IPL の起動・Human68k の起動・個々のゲームではレジスタ仕様の差でつまずく可能性があり、その場合は反復修正が必要です。
- 音の絶対音程は OPM 4MHz クロック基準の計算値。FM の音色再現は簡易実装で、実機/fmgen と完全一致はしません。
- タイミングは行単位（CPU は約 10MHz×設定倍率）。ラスタ割り込みに厳密に依存するソフトは動かない場合があります。
- `px68k_cpuspeed` は即時反映、`px68k_ramsize` はリセット/ロード時に反映。
