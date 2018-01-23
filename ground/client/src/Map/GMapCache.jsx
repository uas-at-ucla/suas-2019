class GMapCache {
  constructor() {
    this.spriteRanges = {
      0: { tl: { x: 0, y: 0 }, br: { x: 0, y: 0 } },
      1: { tl: { x: 1, y: 0 }, br: { x: 1, y: 0 } },
      2: { tl: { x: 2, y: 1 }, br: { x: 2, y: 1 } },
      3: { tl: { x: 4, y: 2 }, br: { x: 4, y: 2 } },
      4: { tl: { x: 9, y: 5 }, br: { x: 9, y: 5 } },
      5: { tl: { x: 18, y: 10 }, br: { x: 18, y: 10 } },
      6: { tl: { x: 36, y: 20 }, br: { x: 36, y: 20 } },
      7: { tl: { x: 73, y: 41 }, br: { x: 73, y: 41 } },
      8: { tl: { x: 147, y: 82 }, br: { x: 147, y: 82 } },
      9: { tl: { x: 294, y: 164 }, br: { x: 295, y: 164 } },
      10: { tl: { x: 589, y: 328 }, br: { x: 591, y: 329 } },
      11: { tl: { x: 1179, y: 657 }, br: { x: 1182, y: 659 } },
      12: { tl: { x: 2358, y: 1315 }, br: { x: 2364, y: 1319 } },
      13: { tl: { x: 4717, y: 2630 }, br: { x: 4728, y: 2639 } },
      14: { tl: { x: 9439, y: 5262 }, br: { x: 9452, y: 5277 } },
      15: { tl: { x: 18878, y: 10525 }, br: { x: 18905, y: 10554 } },
      16: { tl: { x: 37757, y: 21051 }, br: { x: 37810, y: 21108 } },
      17: { tl: { x: 75514, y: 42102 }, br: { x: 75620, y: 42216 } },
      18: { tl: { x: 151028, y: 84204 }, br: { x: 151240, y: 168864 } },
      19: { tl: { x: 302056, y: 168408 }, br: { x: 402480, y: 168864 } },
      20: { tl: { x: 604112, y: 336816 }, br: { x: 804960, y: 33728 } },
      21: { tl: { x: 1208224, y: 673632 }, br: { x: 1609920, y: 67456 } }
    };

    this.max_zoom = 21;

    var webStorage = null;

    if (window.localStorage) {
      webStorage = this.LocalStorageWebStorageImpl();
    } else {
      alert("Your browser don't support localStorage");
    }
  }

  imageToBase64(image) {
    var canvas = document.createElement("canvas");
    canvas.width = image.width;
    canvas.height = image.height;

    var context = canvas.getContext("2d");
    context.drawImage(image, 0, 0);

    return canvas.toDataURL("image/png");
  }

  loadImageToWebStorage(zoom, x, y) {
    var url = "cache/" + zoom + "/" + x + "_" + y + ".png";
    var image = new Image();
    image.onload = function() {
      this.webStorage.setItem(
        [zoom, x, y].join("_"),
        this.imageToBase64(image)
      );
    };
    image.src = url;
  }

  clearWebStorage() {
    this.webStorage.clear();
  }

  prepareWebStorage() {
    for (var zoom in this.spriteRanges) {
      if (zoom > this.max_zoom) {
        break;
      }
      var sprites = this.spriteRanges[zoom];
      for (var x = sprites.tl.x; x <= sprites.br.x; x++) {
        for (var y = sprites.tl.y; y <= sprites.br.y; y++) {
          this.loadImageToWebStorage(zoom, x, y);
        }
      }
    }
  }

  checkTileInSprites(coord, zoom) {
    console.log(zoom);
    var sprites = this.spriteRanges[zoom];
    return (
      sprites.tl.x <= coord.x &&
      coord.x <= sprites.br.x &&
      sprites.tl.y <= coord.y &&
      coord.y <= sprites.br.y
    );
  }

  getOsmTileImgSrc(coord, zoom) {
    return (
      "http://tile.openstreetmap.org/" +
      zoom +
      "/" +
      coord.x +
      "/" +
      coord.y +
      ".png"
    );
  }

  getGmapTileImgSrc(coord, zoom) {
    let tile_src =
      "http://khms0.googleapis.com/kh?v=747&hl=en-US&x=" +
      coord.x +
      "&y=" +
      coord.y +
      "&z=" +
      zoom;
    console.log(tile_src);
    return tile_src;
  }

  getLocalTileImgSrc(coord, zoom) {
    return "cache/" + zoom + "/" + coord.x + "_" + coord.y + ".png";
  }

  getWebStorageTileImgSrc(coord, zoom) {
    return this.webStorage.getItem([zoom, coord.x, coord.y].join("_"));
  }

  LocalStorageWebStorageImpl() {
    this.webStorageType = "localStorage";

    this.getItem = function(name) {
      return localStorage.getItem(name);
    };

    this.setItem = function(name, value) {
      localStorage.setItem(name, value);
    };

    this.clear = function() {
      localStorage.clear();
    };
  }
}
export default GMapCache;
