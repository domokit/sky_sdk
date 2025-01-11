import SwiftUI

struct ContentView: View {
    @State private var items: [String] = Array(repeating: "dash", count: 20)

    var body: some View {
        List {
            ForEach(items, id: \.self) { item in
                Image(item)
                    .resizable()
                    .scaledToFit()
                    .onAppear {
                        if items.last == item {
                            loadMoreItems()
                        }
                    }
            }
        }
    }

    func loadMoreItems() {
        DispatchQueue.main.asyncAfter(deadline: .now() + 1) {
            for _ in 0..<20 { // Add 20 more images
                items.append("dash")
            }
        }
    }
}

#Preview {
    ContentView()
}
