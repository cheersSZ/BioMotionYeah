import SwiftUI

struct RootView: View {
    var body: some View {
        TabView {
            ContentView()
                .tabItem {
                    Label("Online", systemImage: "dot.radiowaves.left.and.right")
                }

            OfflineAnalysisView()
                .tabItem {
                    Label("Offline", systemImage: "film.stack")
                }
        }
    }
}
